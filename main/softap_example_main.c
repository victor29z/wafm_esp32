/*  WiFi softAP + Stepper Motor Control

   Builds on the ESP-IDF softAP example and adds:
   - Stepper control on GPIO5 (STEP), GPIO6 (DIR), GPIO7 (EN)
   - Trapezoidal speed profile (200 ~ 10000 pulses/s)
   - Simple web UI for jog/move/stop/single-step and max speed setting
*/
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

#define GPIO_STEP   5
#define GPIO_DIR    6
#define GPIO_ENABLE 10
#define GPIO_MOTOR_EN 10

#define MIN_SPEED_HZ 200
#define MAX_SPEED_HZ 10000
#define PROFILE_TICK_MS 10
#define ACCEL_STEP_PPS 70 // speed delta per PROFILE_TICK_MS

#define MAX_SAMPLES 1024
#define MAX_LINES 1024

static const char *TAG = "stepper_ap";

typedef enum {
    CMD_MOVE = 0,
    CMD_JOG_START,
    CMD_JOG_STOP,
    CMD_STOP_NOW,
    CMD_SINGLE_STEP,
    CMD_SET_MAX_SPEED
} stepper_cmd_type_t;

typedef struct {
    stepper_cmd_type_t type;
    int32_t value;      // distance pulses for MOVE, dir for jog/step, max speed for set
} stepper_cmd_t;

typedef struct {
    bool running;
    bool jog_mode;
    bool stop_requested;
    int dir;
    int64_t target_pulses;
    int64_t pulses_done;
    int32_t min_speed_hz;
    int32_t max_speed_hz;
    int32_t current_speed_hz;
    int32_t planned_peak_speed;
    int64_t accel_end_pulses;
    int64_t decel_start_pulses;
} stepper_state_t;

static stepper_state_t s_state = {
    .running = false,
    .jog_mode = false,
    .stop_requested = false,
    .dir = 1,
    .target_pulses = 0,
    .pulses_done = 0,
    .min_speed_hz = MIN_SPEED_HZ,
    .max_speed_hz = 4000, // default max speed until user sets
    .current_speed_hz = MIN_SPEED_HZ,
    .planned_peak_speed = MIN_SPEED_HZ,
    .accel_end_pulses = 0,
    .decel_start_pulses = 0,
};

// AFM Scan parameters
static float scan_x_size = 10.0f;
static float scan_y_size = 10.0f;
static float scan_x_offset = 0.0f;
static float scan_y_offset = 0.0f;
static float scan_rate = 1.0f;
static int scan_samples = 256;
static int scan_lines = 256;
static bool scanning = false;
static bool scan_direction_upward = false; // true: upward (from max to 0), false: downward (from 0 to max)
static int current_scan_line = 0;
static bool frame_completed = false;
EXT_RAM_BSS_ATTR static float scan_data[MAX_LINES][MAX_SAMPLES]; // 1024x1024 float array in PSRAM

static esp_timer_handle_t s_step_timer;
static portMUX_TYPE s_state_spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_cmd_queue;
static volatile bool s_step_level = false;

static const char *INDEX_HTML =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Stepper Control & AFM Scan</title>"
    "<style>body{font-family:Arial,Helvetica,sans-serif;background:#0f172a;color:#e2e8f0;padding:16px;}"
    "h1{font-size:20px;margin-bottom:8px;}label{display:block;margin:8px 0 4px;}"
    "input{width:160px;padding:6px;border-radius:6px;border:1px solid #1e293b;background:#0b1220;color:#e2e8f0;}"
    "button{margin:6px 6px 6px 0;padding:10px 14px;border:0;border-radius:8px;background:#22c55e;color:#0b1220;font-weight:600;cursor:pointer;}"
    "button.danger{background:#f43f5e;color:#0b1220;}button.secondary{background:#38bdf8;color:#0b1220;}button.stop{background:#f43f5e;color:#0b1220;}"
    "button:active{transform:scale(0.98);}section{margin-bottom:18px;}canvas{border:1px solid #1e293b;}"
    ".status{font-size:14px;margin:8px 0;}.top{display:flex;}.left,.right{flex:1;margin-right:10px;}"
    "</style></head><body><div class='top'><div class='left'><h1>Stepper Motor Control</h1>"
    "<section><label>Max speed (200-10000 pps)</label><input id='vmax' type='number' min='200' max='10000' value='4000'>"
    "<button onclick='setMaxSpeed()'>Set</button></section>"
    "<section><label>Motor Enable</label>"
    "<button onclick='setMotorEnable(1)'>Enable</button>"
    "<button onclick='setMotorEnable(0)'>Disable</button></section>"
    "<section><label>Move distance (pulses, +/-)</label><input id='distance' type='number' value='1000'>"
    "<button onclick='moveOnce()' class='secondary'>Move</button></section>"
    "<section><button id='jogPos' onmousedown='jog(1,true)' onmouseup='jog(1,false)' ontouchstart='jog(1,true)' ontouchend='jog(1,false)' class='secondary'>Jog +</button>"
    "<button id='jogNeg' onmousedown='jog(-1,true)' onmouseup='jog(-1,false)' ontouchstart='jog(-1,true)' ontouchend='jog(-1,false)' class='secondary'>Jog -</button>"
    "<button onclick='singleStep(1)'>+1 Step</button><button onclick='singleStep(-1)'>-1 Step</button>"
    "<button onclick='stopNow()' class='danger'>Stop</button></section></div><div class='right'><h1>AFM Scan Control</h1>"
    "<section><label>X Scan Size (um)</label><input id='x_scan_size' type='number' min='0' max='30' step='0.1' value='10.0'>"
    "<label>Y Scan Size (um)</label><input id='y_scan_size' type='number' min='0' max='30' step='0.1' value='10.0'>"
    "<label>X Offset (um)</label><input id='x_offset' type='number' min='0' max='30' step='0.1' value='0'>"
    "<label>Y Offset (um)</label><input id='y_offset' type='number' min='0' max='30' step='0.1' value='0'>"
    "<label>Scan Rate (Hz)</label><input id='scan_rate' type='number' min='0' max='10' step='0.1' value='1.0'>"
    "<label>Samples (0-1024)</label><input id='samples' type='number' min='0' max='1024' value='256'>"
    "<label>Lines (0-1024)</label><input id='lines' type='number' min='0' max='1024' value='256'>"
    "<button onclick='updateScanParams()'>Update</button></section>"
    "<section><button id='scanBtn' onclick='toggleScan()' class='secondary'>Scan Start</button>"
    "<button onclick='setDirection(true)'>Upward</button><button onclick='setDirection(false)'>Downward</button></section>"
    "<section><label>Save data after scan</label><input id='saveData' type='checkbox'></section></div></div>"
    "<h1>Height Image</h1>"
    "<section><canvas id='bitmapCanvas' width='1024' height='1024'></canvas></section>"
    "<section><canvas id='curveCanvas' width='400' height='200'></canvas></section>"
    "<div class='status' id='statusBar'>Scanning: Stopped | Direction: Upward | Current Line: 0</div>"
    "<script>const fetchPost=(url,body)=>fetch(url,{method:'POST',headers:{'Content-Type':'text/plain'},body});"
    "function setMaxSpeed(){const v=document.getElementById('vmax').value;fetchPost('/api/max_speed',v);}"
    "function setMotorEnable(val){fetchPost('/api/enable',val);}"
    "function moveOnce(){const d=document.getElementById('distance').value;fetchPost('/api/move',d);}"
    "function jog(dir,start){fetchPost('/api/jog?action='+(start?'start':'stop')+'&dir='+dir,'');}"
    "function stopNow(){fetchPost('/api/stop','');}"
    "function singleStep(dir){fetchPost('/api/step?dir='+dir,'');}"
    "function updateScanParams(){"
    "const params={x_scan_size:parseFloat(document.getElementById('x_scan_size').value),"
    "y_scan_size:parseFloat(document.getElementById('y_scan_size').value),"
    "x_offset:parseFloat(document.getElementById('x_offset').value),"
    "y_offset:parseFloat(document.getElementById('y_offset').value),"
    "scan_rate:parseFloat(document.getElementById('scan_rate').value),"
    "samples:parseInt(document.getElementById('samples').value),"
    "lines:parseInt(document.getElementById('lines').value)};"
    "fetchPost('/api/update_scan',JSON.stringify(params)).then(()=>{updateDisplay();});}"
    "function toggleScan(){fetchPost('/api/scan_toggle','').then(()=>{updateDisplay();});}"
    "function setDirection(up){fetchPost('/api/set_direction',up?'up':'down').then(()=>{updateDisplay();});}"
    "let updateIntervalId = null;"
    "let currentRate = 1;"
    "function setUpdateInterval(rate){"
    "const newRate = Math.max(1, rate);"
    "if (currentRate === newRate && updateIntervalId) return;"
    "if(updateIntervalId){clearInterval(updateIntervalId);}"
    "currentRate = newRate;"
    "updateIntervalId = setInterval(updateDisplay, 1000 / newRate);"
    "}"
    "function updateDisplay(){"
    "fetch('/api/get_scan_data').then(r=>r.json()).then(data=>{"
    "if(data.frame_completed && document.getElementById('saveData').checked){if(confirm('Frame completed. Save data?')){downloadData(dataArray, data.lines, data.samples);}}"
    "updateBitmap(data.current_row, data.current_line, data.samples, data.lines);"
    "updateCurve(data.row0, data.row1, data.samples);"
    "updateStatus(data.scanning, data.direction, data.current_line);"
    "const targetRate = data.scanning && data.scan_rate > 0 ? data.scan_rate : 1;"
    "setUpdateInterval(targetRate);"
    "if(data.scanning){document.getElementById('scanBtn').innerText='Stop';document.getElementById('scanBtn').className='stop';}else{document.getElementById('scanBtn').innerText='Scan Start';document.getElementById('scanBtn').className='secondary';}"
    "}).catch(err=>console.error('scan update failed',err));}"
    "function downloadData(data, lines, samples){"
    "let txt='';for(let y=0;y<lines;y++){for(let x=0;x<samples;x++){txt+=data[y][x].toFixed(3);if(x<samples-1)txt+=' ';}txt+='\\n';}"
    "const blob=new Blob([txt],{type:'text/plain'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download='scan_data.txt';a.click();URL.revokeObjectURL(url);}"
    "let dataArray = [];"
    "function updateBitmap(currentRow, line, samples, lines){"
    "const targetSize = 1024;"
    "if(dataArray.length !== lines || (dataArray[0] && dataArray[0].length !== samples)){dataArray = new Array(lines).fill().map(() => new Array(samples).fill(0));}"
    "dataArray[line] = currentRow;"
    "const canvas=document.getElementById('bitmapCanvas');const ctx=canvas.getContext('2d');"
    "const imgData=ctx.createImageData(targetSize, targetSize);"
    "if(samples <= 0 || lines <= 0){ctx.putImageData(imgData,0,0);return;}"
    "for(let y=0;y<targetSize;y++){"
    "const fy = (lines - 1) * y / (targetSize - 1);"
    "const y0 = Math.floor(fy);"
    "const y1 = Math.min(lines - 1, y0 + 1);"
    "const ty = fy - y0;"
    "for(let x=0;x<targetSize;x++){"
    "const fx = (samples - 1) * x / (targetSize - 1);"
    "const x0 = Math.floor(fx);"
    "const x1 = Math.min(samples - 1, x0 + 1);"
    "const tx = fx - x0;"
    "const v00 = dataArray[y0][x0];"
    "const v10 = dataArray[y0][x1];"
    "const v01 = dataArray[y1][x0];"
    "const v11 = dataArray[y1][x1];"
    "const v0 = v00 + (v10 - v00) * tx;"
    "const v1 = v01 + (v11 - v01) * tx;"
    "const val = v0 + (v1 - v0) * ty;"
    "const gray = Math.max(0, Math.min(255, Math.floor(val * 255)));"
    "const idx = (y * targetSize + x) * 4;"
    "imgData.data[idx] = gray;"
    "imgData.data[idx + 1] = gray;"
    "imgData.data[idx + 2] = gray;"
    "imgData.data[idx + 3] = 255;"
    "}}"
    "ctx.putImageData(imgData,0,0);}"
    "function updateCurve(row0, row1, samples){"
    "const canvas=document.getElementById('curveCanvas');const ctx=canvas.getContext('2d');ctx.clearRect(0,0,canvas.width,canvas.height);"
    "if(samples <= 0) return;"
    "ctx.lineWidth = 2;"
    "ctx.strokeStyle='red';ctx.beginPath();ctx.moveTo(0,(1-row0[0])*canvas.height);for(let i=1;i<samples;i++){const x=i*(canvas.width/samples);const y=(1-row0[i])*canvas.height;ctx.lineTo(x,y);}ctx.stroke();"
    "ctx.strokeStyle='blue';ctx.beginPath();ctx.moveTo(0,(1-row1[0])*canvas.height);for(let i=1;i<samples;i++){const x=i*(canvas.width/samples);const y=(1-row1[i])*canvas.height;ctx.lineTo(x,y);}ctx.stroke();}"
    "function updateStatus(scanning, direction, line){document.getElementById('statusBar').innerText=`Scanning: ${scanning?'Started':'Stopped'} | Direction: ${direction?'Upward':'Downward'} | Current Line: ${line}`;}"
    "setUpdateInterval(1);updateDisplay();"
    "</script></body></html>";

static void set_direction(int dir)
{
    gpio_set_level(GPIO_DIR, dir > 0 ? 1 : 0);
    s_state.dir = (dir >= 0) ? 1 : -1;
}

static void step_enable(bool enable)
{
    gpio_set_level(GPIO_ENABLE, enable ? 0 : 1);
    ESP_LOGI(TAG, "a - Enable set to %d", enable ? 0 : 1);
}

static void IRAM_ATTR step_timer_cb(void *arg)
{
    s_step_level = !s_step_level;
    gpio_set_level(GPIO_STEP, s_step_level);

    if (!s_step_level) {
        return; // count only on rising edge
    }

    bool stop_timer = false;
    portENTER_CRITICAL_ISR(&s_state_spinlock);
    if (s_state.running) {
        s_state.pulses_done++;
        if (!s_state.jog_mode && s_state.pulses_done >= s_state.target_pulses) {
            s_state.running = false;
            stop_timer = true;
        }
        if (s_state.stop_requested && s_state.current_speed_hz <= s_state.min_speed_hz) {
            s_state.running = false;
            stop_timer = true;
        }
    }
    portEXIT_CRITICAL_ISR(&s_state_spinlock);

    if (stop_timer) {
        esp_timer_stop(s_step_timer);
        s_step_level = false;
        gpio_set_level(GPIO_STEP, 0);
    }
}

static void update_timer_freq(int32_t freq_hz)
{
    if (freq_hz < MIN_SPEED_HZ) {
        freq_hz = MIN_SPEED_HZ;
    }
    if (freq_hz > s_state.max_speed_hz) {
        freq_hz = s_state.max_speed_hz;
    }

    int64_t period_us = 1000000LL / freq_hz / 2; // half-period because we toggle each cb
    if (period_us < 20) {
        period_us = 20; // guardrail
    }

    if (esp_timer_is_active(s_step_timer)) {
        esp_timer_stop(s_step_timer);
    }
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_step_timer, period_us));
    s_state.current_speed_hz = freq_hz;
}

static void compute_profile_and_update(void)
{
    stepper_state_t snap;
    portENTER_CRITICAL(&s_state_spinlock);
    snap = s_state;
    portEXIT_CRITICAL(&s_state_spinlock);

    if (!snap.running) {
        return;
    }

    int32_t desired_freq = snap.current_speed_hz;

    if (snap.jog_mode) {
        if (!snap.stop_requested) {
            desired_freq = snap.current_speed_hz + ACCEL_STEP_PPS;
            if (desired_freq > snap.max_speed_hz) desired_freq = snap.max_speed_hz;
        } else {
            desired_freq = snap.current_speed_hz - ACCEL_STEP_PPS;
            if (desired_freq < snap.min_speed_hz) desired_freq = snap.min_speed_hz;
        }
    } else {
        // Pre-planned move profile
        if (snap.pulses_done < snap.accel_end_pulses) {
            desired_freq = snap.current_speed_hz + ACCEL_STEP_PPS;
            if (desired_freq > snap.planned_peak_speed) desired_freq = snap.planned_peak_speed;
        } else if (snap.pulses_done >= snap.decel_start_pulses) {
            desired_freq = snap.current_speed_hz - ACCEL_STEP_PPS;
            if (desired_freq < snap.min_speed_hz) desired_freq = snap.min_speed_hz;
        } else {
            desired_freq = snap.planned_peak_speed;
        }
    }

    if (desired_freq != snap.current_speed_hz) {
        update_timer_freq(desired_freq);
    }
}

static void motion_task(void *arg)
{
    stepper_cmd_t cmd;
    for (;;) {
        if (xQueueReceive(s_cmd_queue, &cmd, pdMS_TO_TICKS(PROFILE_TICK_MS)) == pdTRUE) {
            switch (cmd.type) {
            case CMD_SET_MAX_SPEED:
                if (cmd.value < MIN_SPEED_HZ) {
                    cmd.value = MIN_SPEED_HZ;
                }
                if (cmd.value > MAX_SPEED_HZ) {
                    cmd.value = MAX_SPEED_HZ;
                }
                portENTER_CRITICAL(&s_state_spinlock);
                s_state.max_speed_hz = cmd.value;
                portEXIT_CRITICAL(&s_state_spinlock);
                ESP_LOGI(TAG, "Max speed set to %d pps", cmd.value);
                break;
            case CMD_MOVE: {
                int64_t distance = cmd.value;
                if (distance == 0) {
                    break;
                }
                int dir = distance > 0 ? 1 : -1;
                distance = llabs(distance);

                // Precompute accel/decel pulses using constant acceleration a (pps/s)
                double a = (double)ACCEL_STEP_PPS * 1000.0 / (double)PROFILE_TICK_MS; // pps/s
                double v_min = (double)s_state.min_speed_hz;
                double v_max = (double)s_state.max_speed_hz;
                double dist_d = (double)distance;

                // pulses needed to accel from v_min to v_max: (v_max^2 - v_min^2)/(2a)
                double accel_needed = (v_max * v_max - v_min * v_min) / (2.0 * a);

                int32_t planned_peak = (int32_t)round(v_max);
                double accel_pulses = accel_needed;

                if (2.0 * accel_needed >= dist_d) {
                    // triangular profile; compute achievable peak speed
                    double v_peak = sqrt(v_min * v_min + a * dist_d);
                    if (v_peak > v_max) v_peak = v_max;
                    planned_peak = (int32_t)round(v_peak);
                    accel_pulses = (v_peak * v_peak - v_min * v_min) / (2.0 * a);
                }

                double decel_start = (double)distance - accel_pulses;

                portENTER_CRITICAL(&s_state_spinlock);
                s_state.running = true;
                s_state.jog_mode = false;
                s_state.stop_requested = false;
                s_state.target_pulses = distance;
                s_state.pulses_done = 0;
                s_state.current_speed_hz = s_state.min_speed_hz;
                s_state.planned_peak_speed = planned_peak;
                s_state.accel_end_pulses = (int64_t)ceil(accel_pulses);
                s_state.decel_start_pulses = (int64_t)floor(decel_start);
                portEXIT_CRITICAL(&s_state_spinlock);

                set_direction(dir);
                step_enable(true);
                update_timer_freq(s_state.min_speed_hz);
                break;
            }
            case CMD_JOG_START: {
                portENTER_CRITICAL(&s_state_spinlock);
                s_state.running = true;
                s_state.jog_mode = true;
                s_state.stop_requested = false;
                s_state.target_pulses = INT64_MAX;
                s_state.pulses_done = 0;
                s_state.current_speed_hz = s_state.min_speed_hz;
                portEXIT_CRITICAL(&s_state_spinlock);

                set_direction(cmd.value >= 0 ? 1 : -1);
                step_enable(true);
                update_timer_freq(s_state.min_speed_hz);
                break;
            }
            case CMD_JOG_STOP:
                portENTER_CRITICAL(&s_state_spinlock);
                s_state.stop_requested = true;
                portEXIT_CRITICAL(&s_state_spinlock);
                break;
            case CMD_STOP_NOW:
                ESP_ERROR_CHECK(esp_timer_stop(s_step_timer));
                portENTER_CRITICAL(&s_state_spinlock);
                s_state.running = false;
                s_state.jog_mode = false;
                s_state.stop_requested = false;
                s_state.pulses_done = 0;
                s_state.planned_peak_speed = s_state.min_speed_hz;
                s_state.accel_end_pulses = 0;
                s_state.decel_start_pulses = 0;
                portEXIT_CRITICAL(&s_state_spinlock);
                s_step_level = false;
                gpio_set_level(GPIO_STEP, 0);
                step_enable(false);
                break;
            case CMD_SINGLE_STEP:
                set_direction(cmd.value >= 0 ? 1 : -1);
                gpio_set_level(GPIO_STEP, 1);
                esp_rom_delay_us(20);
                gpio_set_level(GPIO_STEP, 0);
                break;
            default:
                break;
            }
        }

        compute_profile_and_update();
    }
}

static esp_err_t parse_int_from_body(httpd_req_t *req, int32_t *out)
{
    char buf[32] = {0};
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(buf)) {
        return ESP_FAIL;
    }
    int recv_len = httpd_req_recv(req, buf, total);
    if (recv_len <= 0) {
        return ESP_FAIL;
    }
    *out = strtol(buf, NULL, 10);
    return ESP_OK;
}

// forward declarations
static void motor_enable_control(bool enable);

static esp_err_t index_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t max_speed_handler(httpd_req_t *req)
{
    int32_t v = 0;
    if (parse_int_from_body(req, &v) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    }
    stepper_cmd_t cmd = {.type = CMD_SET_MAX_SPEED, .value = v};
    xQueueSend(s_cmd_queue, &cmd, 0);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t enable_handler(httpd_req_t *req)
{
    int32_t v = 0;
    if (parse_int_from_body(req, &v) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    }
    motor_enable_control(v != 0);
    
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t move_handler(httpd_req_t *req)
{
    int32_t d = 0;
    if (parse_int_from_body(req, &d) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    }
    stepper_cmd_t cmd = {.type = CMD_MOVE, .value = d};
    xQueueSend(s_cmd_queue, &cmd, 0);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t jog_handler(httpd_req_t *req)
{
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no query");
    }
    char action[16];
    char dir_str[16];
    if (httpd_query_key_value(query, "action", action, sizeof(action)) != ESP_OK ||
        httpd_query_key_value(query, "dir", dir_str, sizeof(dir_str)) != ESP_OK) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing params");
    }
    int dir = atoi(dir_str);
    if (strcmp(action, "start") == 0) {
        stepper_cmd_t cmd = {.type = CMD_JOG_START, .value = dir};
        xQueueSend(s_cmd_queue, &cmd, 0);
    } else {
        stepper_cmd_t cmd = {.type = CMD_JOG_STOP, .value = 0};
        xQueueSend(s_cmd_queue, &cmd, 0);
    }
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t stop_handler(httpd_req_t *req)
{
    stepper_cmd_t cmd = {.type = CMD_STOP_NOW, .value = 0};
    xQueueSend(s_cmd_queue, &cmd, 0);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t step_handler(httpd_req_t *req)
{
    char query[32];
    int dir = 1;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char dir_str[16];
        if (httpd_query_key_value(query, "dir", dir_str, sizeof(dir_str)) == ESP_OK) {
            dir = atoi(dir_str) >= 0 ? 1 : -1;
        }
    }
    stepper_cmd_t cmd = {.type = CMD_SINGLE_STEP, .value = dir};
    xQueueSend(s_cmd_queue, &cmd, 0);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t update_scan_handler(httpd_req_t *req)
{
    char buf[512] = {0};
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(buf)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    }
    int recv_len = httpd_req_recv(req, buf, total);
    if (recv_len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
    }
    // Simple JSON parse
    float x_size, y_size, x_off, y_off, rate;
    int samp, lin;
    sscanf(buf, "{\"x_scan_size\":%f,\"y_scan_size\":%f,\"x_offset\":%f,\"y_offset\":%f,\"scan_rate\":%f,\"samples\":%d,\"lines\":%d}",
           &x_size, &y_size, &x_off, &y_off, &rate, &samp, &lin);
    // Validate ranges
    if (x_size < 0 || x_size > 30) x_size = 10.0f;
    if (y_size < 0 || y_size > 30) y_size = 10.0f;
    if (x_off < 0 || x_off > 30) x_off = 0.0f;
    if (y_off < 0 || y_off > 30) y_off = 0.0f;
    if (rate < 0 || rate > 10) rate = 1.0f;
    if (samp < 0 || samp > 1024) samp = 256;
    if (lin < 0 || lin > 1024) lin = 256;
    scan_x_size = x_size;
    scan_y_size = y_size;
    scan_x_offset = x_off;
    scan_y_offset = y_off;
    scan_rate = rate;
    scan_samples = samp;
    scan_lines = lin;
    ESP_LOGI(TAG, "Scan params updated: x_size=%.1f, y_size=%.1f, x_offset=%.1f, y_offset=%.1f, rate=%.1f, samples=%d, lines=%d", scan_x_size, scan_y_size, scan_x_offset, scan_y_offset, scan_rate, scan_samples, scan_lines);
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t scan_toggle_handler(httpd_req_t *req)
{
    scanning = !scanning;
    if (scanning) {
        if (scan_direction_upward) {
            current_scan_line = scan_lines - 1; // start from max
        } else {
            current_scan_line = 0; // start from 0
        }
        frame_completed = false;
        // Initialize first two rows with random data
        for (int i = 0; i < scan_samples; i++) {
            scan_data[0][i] = (float)rand() / RAND_MAX;
            scan_data[1][i] = (float)rand() / RAND_MAX;
        }
    }
    ESP_LOGI(TAG, "Scanning %s", scanning ? "started" : "stopped");
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t set_direction_handler(httpd_req_t *req)
{
    char buf[16] = {0};
    int total = req->content_len;
    if (total <= 0 || total >= (int)sizeof(buf)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    }
    int recv_len = httpd_req_recv(req, buf, total);
    if (recv_len <= 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "recv failed");
    }
    if (strcmp(buf, "up") == 0) {
        scan_direction_upward = true;
        ESP_LOGI(TAG, "Scan direction: Upward");
    } else if (strcmp(buf, "down") == 0) {
        scan_direction_upward = false;
        ESP_LOGI(TAG, "Scan direction: Downward");
    }
    return httpd_resp_sendstr(req, "ok");
}

static esp_err_t get_scan_data_handler(httpd_req_t *req)
{
    int adjacent_line = current_scan_line;
    if (scan_direction_upward) {
        if (current_scan_line + 1 < scan_lines) {
            adjacent_line = current_scan_line + 1;
        }
    } else {
        if (current_scan_line - 1 >= 0) {
            adjacent_line = current_scan_line - 1;
        }
    }

    char *json_str = malloc(64 * 1024); // 64KB buffer
    if (!json_str) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    }
    char *p = json_str;
    p += sprintf(p, "{\"scanning\":%s,\"direction\":%s,\"current_line\":%d,\"samples\":%d,\"lines\":%d,\"scan_rate\":%.1f,\"frame_completed\":%s,\"row0\":[",
                scanning ? "true" : "false", scan_direction_upward ? "true" : "false", current_scan_line, scan_samples, scan_lines, scan_rate, frame_completed ? "true" : "false");
    for (int x = 0; x < scan_samples; x++) {
        p += sprintf(p, "%.3f", scan_data[current_scan_line][x]);
        if (x < scan_samples - 1) p += sprintf(p, ",");
    }
    p += sprintf(p, "],\"row1\":[");
    for (int x = 0; x < scan_samples; x++) {
        p += sprintf(p, "%.3f", scan_data[adjacent_line][x]);
        if (x < scan_samples - 1) p += sprintf(p, ",");
    }
    p += sprintf(p, "],\"current_row\":[");
    for (int x = 0; x < scan_samples; x++) {
        p += sprintf(p, "%.3f", scan_data[current_scan_line][x]);
        if (x < scan_samples - 1) p += sprintf(p, ",");
    }
    p += sprintf(p, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    frame_completed = false; // reset after sending
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 12;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL};
        httpd_uri_t max_speed_uri = {.uri = "/api/max_speed", .method = HTTP_POST, .handler = max_speed_handler, .user_ctx = NULL};
        httpd_uri_t move_uri = {.uri = "/api/move", .method = HTTP_POST, .handler = move_handler, .user_ctx = NULL};
        httpd_uri_t jog_uri = {.uri = "/api/jog", .method = HTTP_POST, .handler = jog_handler, .user_ctx = NULL};
        httpd_uri_t stop_uri = {.uri = "/api/stop", .method = HTTP_POST, .handler = stop_handler, .user_ctx = NULL};
        httpd_uri_t step_uri = {.uri = "/api/step", .method = HTTP_POST, .handler = step_handler, .user_ctx = NULL};
        httpd_uri_t enable_uri = {.uri = "/api/enable", .method = HTTP_POST, .handler = enable_handler, .user_ctx = NULL};
        httpd_uri_t update_scan_uri = {.uri = "/api/update_scan", .method = HTTP_POST, .handler = update_scan_handler, .user_ctx = NULL};
        httpd_uri_t scan_toggle_uri = {.uri = "/api/scan_toggle", .method = HTTP_POST, .handler = scan_toggle_handler, .user_ctx = NULL};
        httpd_uri_t set_direction_uri = {.uri = "/api/set_direction", .method = HTTP_POST, .handler = set_direction_handler, .user_ctx = NULL};
        httpd_uri_t get_scan_data_uri = {.uri = "/api/get_scan_data", .method = HTTP_GET, .handler = get_scan_data_handler, .user_ctx = NULL};

        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &max_speed_uri);
        httpd_register_uri_handler(server, &move_uri);
        httpd_register_uri_handler(server, &jog_uri);
        httpd_register_uri_handler(server, &stop_uri);
        httpd_register_uri_handler(server, &step_uri);
        httpd_register_uri_handler(server, &enable_uri);
        httpd_register_uri_handler(server, &update_scan_uri);
        httpd_register_uri_handler(server, &scan_toggle_uri);
        httpd_register_uri_handler(server, &set_direction_uri);
        httpd_register_uri_handler(server, &get_scan_data_uri);
    }
    return server;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                .required = true,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            .gtk_rekey_interval = EXAMPLE_GTK_REKEY_INTERVAL,
        },
    };

    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "softAP ready. SSID:%s password:%s channel:%d", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

static void stepper_gpio_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_STEP) | (1ULL << GPIO_DIR) | (1ULL << GPIO_ENABLE) | (1ULL << GPIO_MOTOR_EN),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_STEP, 0);
    gpio_set_level(GPIO_DIR, 0);
    gpio_set_level(GPIO_ENABLE, 1); // disable by default (assuming low-enable)
    gpio_set_level(GPIO_MOTOR_EN, 0); // motor disabled by default (high = enable)
}

static void scan_task(void *arg)
{
    while (1) {
        if (scanning) {
            // Generate data for current line
            for (int i = 0; i < scan_samples; i++) {
                scan_data[current_scan_line][i] = (float)rand() / RAND_MAX;
            }
            ESP_LOGI(TAG, "Scan line %d completed", current_scan_line);

            // Update current line
            if (scan_direction_upward) {
                current_scan_line--;
                if (current_scan_line < 0) {
                    current_scan_line = 0;
                    scan_direction_upward = false; // switch to downward
                    frame_completed = true;
                }
            } else {
                current_scan_line++;
                if (current_scan_line >= scan_lines) {
                    current_scan_line = scan_lines - 1;
                    scan_direction_upward = true; // switch to upward
                    frame_completed = true;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / scan_rate)); // delay based on scan rate
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    stepper_gpio_init();

    const esp_timer_create_args_t timer_args = {
        .callback = &step_timer_cb,
        .name = "stepper"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &s_step_timer));

    s_cmd_queue = xQueueCreate(8, sizeof(stepper_cmd_t));
    xTaskCreatePinnedToCore(motion_task, "motion_task", 4096, NULL, 6, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(scan_task, "scan_task", 4096, NULL, 5, NULL, tskNO_AFFINITY);

    wifi_init_softap();
    start_webserver();

    // Initialize scan data with random values
    for (int y = 0; y < MAX_LINES; y++) {
        for (int x = 0; x < MAX_SAMPLES; x++) {
            scan_data[y][x] = (float)rand() / RAND_MAX;
        }
    }

    ESP_LOGI(TAG, "Stepper controller ready. Open http://192.168.4.1/ in browser");
}

static void motor_enable_control(bool enable)
{
    gpio_set_level(GPIO_MOTOR_EN, enable ? 0 : 1);
    ESP_LOGI(TAG, "Enable set to %d", enable ? 0 : 1);
}
