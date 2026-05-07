# elegoo-visual-navigation

这是用于 ELEGOO Smart Robot Car 的视觉导航软件。机器人通过 ESP32 摄像头视频流检测 ArUco 目标码，Raspberry Pi 估计目标方向和距离，然后通过串口向 Arduino Uno 发送车轮 PWM 指令，由 Arduino 控制电机并读取车载传感器。

Raspberry Pi 运行滚动时域控制器。它用摄像头舵机搜索未访问目标，选择当前可见目标中估计行驶代价最低的目标，转向该目标，用 PID 跟踪目标，并在每次访问完成后重新规划。所有配置目标访问完成后，机器人停止。

英文版：[README.md](README.md)

## 系统组成

- ELEGOO Smart Robot Car 四轮驱动底盘
- Arduino Uno，用于电机、舵机、超声波传感器和 MPU6050 偏航角积分
- 安装在舵机上的 ESP32 摄像头模块
- Raspberry Pi，用于视觉检测和导航控制
- 打印的 ArUco 目标码

数据流：

1. ESP32 在 `http://<esp32-ip>/stream` 提供 MJPEG 摄像头视频流。
2. Raspberry Pi 读取视频流，检测目标码，并计算转向和速度。
3. Raspberry Pi 通过串口向 Arduino 发送 `MOTOR <left_pwm> <right_pwm>` 指令。
4. Arduino 输出 `SENSOR <yaw_deg> <distance_cm>` 传感器读数，并执行电机或舵机指令。

控制器模式包括 `stop`、`searching`、`turning` 和 `tracking`。

## 仓库结构

- `elegoo/`：Arduino Uno 固件。`elegoo.ino` 包含主循环、串口协议、电机超时、超声波读取、舵机指令和陀螺仪偏航角跟踪。
- `esp32/`：ESP32 摄像头固件。`esp32.ino` 启动 Wi-Fi 和摄像头服务器；`app_httpd.cpp` 提供实时视频流。
- `raspberry/`：Raspberry Pi 软件。`main.py` 是主程序，`navigation.py` 负责运行循环，`controller.py` 选择目标和模式，`vision.py` 处理视频流和 ArUco 检测，`motor_mixer.py` 将机器人运动指令转换为车轮 PWM，`arduino_io.py` 处理串口通信。
- `test/`：独立测试草图和脚本。

## Raspberry Pi 设置

在 Raspberry Pi 上安装 Python 依赖：

```bash
cd raspberry
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pyserial opencv-contrib-python
```

从 `raspberry/` 目录运行导航程序，这样 `main.py` 可以加载 `camera_calibration.npz`：

```bash
python3 main.py
```

在预览窗口中按 `q` 或 `Esc` 停止程序。

## 摄像头标定

`main.py` 需要 `raspberry/camera_calibration.npz`。使用 9x6 内角点棋盘格照片运行标定脚本生成该文件：

```bash
cd raspberry
python3 camera_calibrate.py "calibration/*.jpg" --square-size-mm 23
```

标定照片应使用与导航运行时相同的摄像头分辨率、焦距和镜头设置。默认输出路径是 `raspberry/camera_calibration.npz`。

## 配置

运行前编辑 `raspberry/settings.py` 中的 `Settings` 数据类：

- `serial_port`：Arduino 串口设备，默认 `/dev/ttyUSB0`
- `stream_url`：ESP32 MJPEG 视频流 URL
- `target_marker_ids`：按访问顺序排列的 ArUco 标记 ID
- `aruco_dictionary_name`：默认 `DICT_4X4_50`
- `marker_size_m`：打印目标码边长，单位为米，默认 `0.05`
- `target_distance_m`：机器人与目标码的停止距离，默认 `0.45`
- `target_distance_tolerance_m`：判定为已到达的距离误差范围，默认 `0.03`
- PID 参数：`heading_kp`、`heading_ki`、`heading_kd`、`distance_kp`、`distance_ki`、`distance_kd`。航向 PID 使用弧度；预览和串口角度仍使用度。
- `target_search_delay_s`：目标连续丢失多久后开始舵机搜索
- `search_servo_step_deg`：舵机搜索的步进角度
- `search_servo_dwell_s`：舵机在每个搜索角度停留的时间
- `servo_center_angle_deg`：摄像头正前方对应的舵机角度。默认 `72.0`，因此 `0` 表示右前方，`180` 表示左后方。
- `search_turn_tolerance_deg`：搜索阶段找到目标后，机器人原地对准时使用的偏航误差结束阈值
- `show_preview`：OpenCV 预览窗口

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

## ESP32 摄像头固件

在 Arduino IDE 中打开 `esp32/esp32.ino`。按 `esp32/notes.txt` 设置开发板选项：

- Board：`ESP32S3 Dev Module`
- USB CDC On Boot：`Enabled`
- Flash Size：`8MB(64Mb)`
- Partition Scheme：`8M with spiffs (3MB APP/1.5MB SPIFFS)`
- PSRAM：`OPI PSRAM`

在 `esp32/esp32.hpp` 中将 `WifiSettings::mode` 设置为 `WifiMode::AccessPoint` 或 `WifiMode::Station`。接入点模式下，将 Raspberry Pi 连接到 `WifiSettings::accessPointSsid` 配置的 ESP32 网络，并将 `raspberry/settings.py` 的 `Settings.stream_url` 设置为 `http://192.168.4.1/stream`。站点模式下，设置 `WifiSettings::stationSsid` 和 `WifiSettings::stationPassword`，上传固件，在该 Wi-Fi 网络中找到 ESP32 的 IP 地址，并将 `Settings.stream_url` 设置为 `http://<esp32-ip>/stream`。

## 运行清单

1. 将 `elegoo/elegoo.ino` 上传到 Arduino Uno。
2. 将 `esp32/esp32.ino` 上传到 ESP32 摄像头板。
3. 生成或复制 `camera_calibration.npz` 到 `raspberry/`。
4. 用 USB 将 Arduino Uno 连接到 Raspberry Pi。
5. 将 Raspberry Pi 连接到 ESP32 接入点，或在站点模式下将 Raspberry Pi 和 ESP32 连接到同一个 Wi-Fi 网络。
6. 更新 `raspberry/settings.py` 中的 `Settings`。
7. 从 `raspberry/` 目录运行 `python3 main.py`。
