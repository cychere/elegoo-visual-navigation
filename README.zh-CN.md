# elegoo-visual-navigation

这是用于 ELEGOO Smart Robot Car 的视觉导航软件。机器人通过 ESP32 摄像头视频流检测 ArUco 目标码，Raspberry Pi 估计目标方向和距离，然后通过串口向 Arduino Uno 发送车轮 PWM 指令，由 Arduino 控制电机并读取车载传感器。

Raspberry Pi 会按配置顺序访问 ArUco 标记 ID。对每个目标，它先用摄像头舵机扫描，直到看到下一个目标码；随后让底盘转向观测到的目标方向，用 PID 控制跟踪目标，并在到达配置的停止距离后开始搜索下一个目标。最后一个配置目标到达后，机器人停止。

English version: [README.md](README.md)

## 系统组成

- ELEGOO Smart Robot Car 四轮驱动底盘
- Arduino Uno，用于电机输出、舵机输出、超声波测距和 MPU6050 偏航角积分
- 安装在舵机上的 ESP32 摄像头模块
- Raspberry Pi，用于摄像头视频流处理、ArUco 检测、目标顺序控制和导航控制
- 打印的 ArUco 目标码

数据流：

1. ESP32 在 `http://<esp32-ip>/stream` 提供 MJPEG 视频流。
2. Raspberry Pi 读取视频流，检测配置的 ArUco 标记，估计当前目标的角度和距离，并计算机器人运动指令。
3. Raspberry Pi 通过串口向 Arduino 发送 `MOTOR <left_pwm> <right_pwm>` 和 `SERVO <angle_deg>` 指令。
4. Arduino 输出 `SENSOR <yaw_deg> <distance_cm>` 传感器读数，并执行电机或舵机指令。

控制器模式包括 `searching`、`turning`、`tracking` 和 `stop`。

## 仓库结构

- `elegoo/`：Arduino Uno 固件。`elegoo.ino` 包含主循环、串口协议、电机超时、超声波读取、舵机指令和陀螺仪偏航角跟踪。`motor.cpp`、`servo.cpp`、`ultrasonic.cpp` 和 `mpu.cpp` 是组件驱动。
- `esp32/`：ESP32 摄像头固件。`esp32.ino` 启动摄像头 Web 服务器并更新状态 LED，`esp32.cpp` 配置摄像头和 Wi-Fi 模式，`app_httpd.cpp` 提供 MJPEG 视频流，`esp32.hpp` 保存 Wi-Fi 设置。
- `raspberry/`：Raspberry Pi 软件。`main.py` 启动程序，`navigation.py` 负责运行循环，`controller.py` 负责目标顺序和模式切换，`vision.py` 处理视频流读取和 ArUco 测量，`motor_mixer.py` 将机器人运动指令转换为车轮 PWM，`arduino_io.py` 处理串口通信，`pid.py` 实现 PID 控制，`settings.py` 保存运行配置。
- `raspberry/camera_calibration.npz`：当前默认加载的摄像头标定文件；从 `raspberry/` 目录运行程序时会使用它。

## Raspberry Pi 设置

在 Raspberry Pi 上安装 Python 依赖：

```bash
cd raspberry
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pyserial opencv-contrib-python
```

从 `raspberry/` 目录运行导航程序，这样默认标定路径会解析到 `raspberry/camera_calibration.npz`：

```bash
python3 main.py
```

在 OpenCV 预览窗口中按 `q` 或 `Esc` 停止程序。

## 摄像头标定

`main.py` 需要 `raspberry/camera_calibration.npz`。当摄像头、焦距、分辨率或镜头位置变化时，应重新生成该文件。

使用 9x6 内角点棋盘格照片运行标定脚本：

```bash
cd raspberry
python3 camera_calibrate.py "calibration/*.jpg" --square-size-mm <measured square size>
```

## 配置

运行前编辑 `raspberry/settings.py` 中的 `Settings` 数据类：

- `serial_port` 和 `baud_rate`：Arduino 串口连接。
- `stream_url` 和 `stream_timeout_s`：ESP32 MJPEG 视频流 URL 和连接超时。
- `target_marker_ids`：按顺序访问的 ArUco 标记 ID。
- `aruco_dictionary_name`：OpenCV ArUco 字典名称。
- `marker_size_m`：打印目标码边长；设为 `None` 时只使用方向测量。
- `camera_calibration_path`：`.npz` 标定文件路径；相对路径会按当前工作目录解析。
- `camera_forward_offset_m` 和 `camera_left_offset_m`：摄像头相对机器人参考点的偏移量。
- `min_area_px`：导航循环使用的最小标记检测面积。
- `target_distance_m` 和 `target_distance_tolerance_m`：目标停止距离和允许距离误差。
- `heading_kp`、`heading_ki`、`heading_kd`：航向 PID 参数。航向控制内部使用弧度。
- `distance_kp`、`distance_ki`、`distance_kd`：距离 PID 参数。
- `target_search_delay_s`：跟踪阶段目标丢失多久后重新开始搜索。
- `search_servo_step_deg` 和 `search_servo_dwell_s`：舵机扫描步进角度和每步停留时间。
- `servo_center_angle_deg`：摄像头正前方对应的舵机角度。
- `search_turn_tolerance_deg`：搜索阶段发现目标后，原地转向结束所使用的偏航误差阈值。
- `show_preview`：启用或关闭 OpenCV 预览窗口。

## Arduino 固件

在 Arduino IDE 中打开 `elegoo/elegoo.ino`，并上传到 Arduino Uno。

需要的 Arduino 库：

- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `Servo`

串口协议波特率为 `115200`：

- Arduino 到 Raspberry Pi：`SENSOR <yaw_deg> <distance_cm>`
- Raspberry Pi 到 Arduino：`MOTOR <left_pwm> <right_pwm>`
- Raspberry Pi 到 Arduino：`SERVO <angle_deg>`

Arduino 循环会更新偏航角、读取超声波距离、输出最新传感器行、处理串口指令，并在没有持续收到电机指令时停止电机。

## ESP32 摄像头固件

在已安装 ESP32 board support 的 Arduino IDE 中打开 `esp32/esp32.ino`。设置开发板选项：

- Board：`ESP32S3 Dev Module`
- USB CDC On Boot：`Enabled`
- Flash Size：`8MB(64Mb)`
- Partition Scheme：`8M with spiffs (3MB APP/1.5MB SPIFFS)`
- PSRAM：`OPI PSRAM`

在 `esp32/esp32.hpp` 中设置 `WifiSettings::mode`：

- `WifiMode::AccessPoint`：ESP32 创建配置的接入点。将 Raspberry Pi 连接到该网络；如果未修改接入点 IP，将 `Settings.stream_url` 设置为 `http://192.168.4.1/stream`。
- `WifiMode::Station`：ESP32 连接到配置的 Wi-Fi 网络。设置 `WifiSettings::stationSsid` 和 `WifiSettings::stationPassword`，上传固件，从串口输出读取 ESP32 IP 地址，然后将 `Settings.stream_url` 设置为 `http://<esp32-ip>/stream`。

## 运行清单

1. 将 `elegoo/elegoo.ino` 上传到 Arduino Uno。
2. 将 `esp32/esp32.ino` 上传到 ESP32 摄像头板。
3. 生成或确认 `raspberry/camera_calibration.npz`。
4. 用 USB 将 Arduino Uno 连接到 Raspberry Pi。
5. 将 Raspberry Pi 连接到 ESP32 接入点，或在站点模式下将 Raspberry Pi 和 ESP32 连接到同一个 Wi-Fi 网络。
6. 更新 `raspberry/settings.py` 中的 `Settings`。
7. 从 `raspberry/` 目录运行 `python3 main.py`。
