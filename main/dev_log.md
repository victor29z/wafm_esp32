# key event handlers
## 1. update_scan_handler will be called when Update button is clicked
## 2. set_direction_handler will be called when Upward or Downward button is clicked
## 3. scan_toggle_handler will be called when Scan Start/Stop button is clicked

# 当前程序框架中，扫描进程由scan_task驱动，该任务中：
### 1. 随机产生虚拟扫描数据
### 2. 行号、方向切换
### 3. 根据scan rate产生延迟控制扫描进程


