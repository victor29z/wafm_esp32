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

static esp_timer_handle_t s_step_timer;
static portMUX_TYPE s_state_spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t s_cmd_queue;
static volatile bool s_step_level = false;

static const char *INDEX_HTML =
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Stepper Control</title>"
    "<style>body{font-family:Arial,Helvetica,sans-serif;background:#0f172a;color:#e2e8f0;padding:16px;}"
    "h1{font-size:20px;margin-bottom:8px;}label{display:block;margin:8px 0 4px;}"
    "input{width:160px;padding:6px;border-radius:6px;border:1px solid #1e293b;background:#0b1220;color:#e2e8f0;}"
    "button{margin:6px 6px 6px 0;padding:10px 14px;border:0;border-radius:8px;background:#22c55e;color:#0b1220;font-weight:600;cursor:pointer;}"
    "button.danger{background:#f43f5e;color:#0b1220;}button.secondary{background:#38bdf8;color:#0b1220;}"
    "button:active{transform:scale(0.98);}section{margin-bottom:18px;}"
    "</style></head><body><h1>Stepper Motor Control</h1>"
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
    "<button onclick='stopNow()' class='danger'>Stop</button></section>"
    "<script>const fetchPost=(url,body)=>fetch(url,{method:'POST',headers:{'Content-Type':'text/plain'},body});"
    "function setMaxSpeed(){const v=document.getElementById('vmax').value;fetchPost('/api/max_speed',v);}"
    "function setMotorEnable(val){fetchPost('/api/enable',val);}"
    "function moveOnce(){const d=document.getElementById('distance').value;fetchPost('/api/move',d);}"
    "function jog(dir,start){fetchPost('/api/jog?action='+(start?'start':'stop')+'&dir='+dir,'');}"
    "function stopNow(){fetchPost('/api/stop','');}"
    "function singleStep(dir){fetchPost('/api/step?dir='+dir,'');}"
    "</script></body></html>";

static void set_direction(int dir)
{
    gpio_set_level(GPIO_DIR, dir > 0 ? 1 : 0);
    s_state.dir = (dir >= 0) ? 1 : -1;
}

static void step_enable(bool enable)
{
    // Active-low enable assumed; adjust here if your driver differs
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

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 8;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {.uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL};
        httpd_uri_t max_speed_uri = {.uri = "/api/max_speed", .method = HTTP_POST, .handler = max_speed_handler, .user_ctx = NULL};
        httpd_uri_t move_uri = {.uri = "/api/move", .method = HTTP_POST, .handler = move_handler, .user_ctx = NULL};
        httpd_uri_t jog_uri = {.uri = "/api/jog", .method = HTTP_POST, .handler = jog_handler, .user_ctx = NULL};
        httpd_uri_t stop_uri = {.uri = "/api/stop", .method = HTTP_POST, .handler = stop_handler, .user_ctx = NULL};
        httpd_uri_t step_uri = {.uri = "/api/step", .method = HTTP_POST, .handler = step_handler, .user_ctx = NULL};
        httpd_uri_t enable_uri = {.uri = "/api/enable", .method = HTTP_POST, .handler = enable_handler, .user_ctx = NULL};

        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &max_speed_uri);
        httpd_register_uri_handler(server, &move_uri);
        httpd_register_uri_handler(server, &jog_uri);
        httpd_register_uri_handler(server, &stop_uri);
        httpd_register_uri_handler(server, &step_uri);
        httpd_register_uri_handler(server, &enable_uri);
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

    wifi_init_softap();
    start_webserver();

    ESP_LOGI(TAG, "Stepper controller ready. Open http://192.168.4.1/ in browser");
}

static void motor_enable_control(bool enable)
{
    gpio_set_level(GPIO_MOTOR_EN, enable ? 0 : 1);
    ESP_LOGI(TAG, "Enable set to %d", enable ? 0 : 1);
}
