# elegoo-visual-navigation

Visual navigation software for an ELEGOO Smart Robot Car. The robot uses an ESP32 camera stream to detect ArUco target markers, estimates marker bearing and distance on a Raspberry Pi, then sends wheel PWM commands to an Arduino Uno that drives the motors and reads onboard sensors.

The Raspberry Pi visits configured ArUco marker IDs in sequence. For each target, it sweeps the camera servo until the next marker is seen, turns the chassis toward the observed bearing, tracks the marker with PID control, and starts the next search after reaching the configured standoff distance. After the final configured target is reached, the robot stops.

Simplified Chinese version: [README.zh-CN.md](README.zh-CN.md)

## System

- ELEGOO Smart Robot Car four-wheel-drive chassis
- Arduino Uno for motor output, servo output, ultrasonic distance sensing, and MPU6050 yaw integration
- ESP32 camera module mounted on the servo
- Raspberry Pi for camera-stream processing, ArUco detection, target sequencing, and navigation control
- Printed ArUco target markers

Data flow:

1. The ESP32 serves an MJPEG stream at `http://<esp32-ip>/stream`.
2. The Raspberry Pi reads the stream, detects configured ArUco markers, estimates the active target angle and distance, and computes robot motion commands.
3. The Raspberry Pi sends `MOTOR <left_pwm> <right_pwm>` and `SERVO <angle_deg>` commands to the Arduino over serial.
4. The Arduino prints `SENSOR <yaw_deg> <distance_cm>` readings and applies the motor or servo command.

Controller modes are `searching`, `turning`, `tracking`, and `stop`.

## Repository Layout

- `elegoo/`: Arduino Uno firmware. `elegoo.ino` owns the main loop, serial protocol, motor timeout, ultrasonic reads, servo commands, and gyro yaw tracking. `motor.cpp`, `servo.cpp`, `ultrasonic.cpp`, and `mpu.cpp` contain the component drivers.
- `esp32/`: ESP32 camera firmware. `esp32.ino` starts the camera web server and updates the status LED, `esp32.cpp` configures the camera and Wi-Fi mode, `app_httpd.cpp` serves the MJPEG stream, and `esp32.hpp` contains Wi-Fi settings.
- `raspberry/`: Raspberry Pi software. `main.py` starts the program, `navigation.py` owns the runtime loop, `controller.py` sequences targets and modes, `vision.py` handles stream reading and ArUco measurement, `motor_mixer.py` converts robot commands to wheel PWM, `arduino_io.py` owns the serial link, `pid.py` implements PID control, and `settings.py` contains runtime configuration.
- `raspberry/camera_calibration.npz`: Current camera calibration file loaded by default when running from `raspberry/`.

## Raspberry Pi Setup

Install the Python dependencies on the Raspberry Pi:

```bash
cd raspberry
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pyserial opencv-contrib-python
```

Run navigation from the `raspberry/` directory so the default calibration path resolves to `raspberry/camera_calibration.npz`:

```bash
python3 main.py
```

Press `q` or `Esc` in the OpenCV preview window to stop.

## Camera Calibration

`main.py` expects `raspberry/camera_calibration.npz`. Regenerate it when the camera, focus, resolution, or lens position changes.

Use photos of a 9x6 inner-corner chessboard:

```bash
cd raspberry
python3 camera_calibrate.py "calibration/*.jpg" --square-size-mm <measured square size>
```

## Configuration

Edit the `Settings` dataclass in `raspberry/settings.py` before running:

- `serial_port` and `baud_rate`: Arduino serial connection.
- `stream_url` and `stream_timeout_s`: ESP32 MJPEG stream URL and connection timeout.
- `target_marker_ids`: ArUco marker IDs to visit sequentially.
- `aruco_dictionary_name`: OpenCV ArUco dictionary name.
- `marker_size_m`: printed marker side length; set it to `None` to use bearing-only measurement.
- `camera_calibration_path`: path to the `.npz` calibration file, relative to the current working directory unless absolute.
- `camera_forward_offset_m` and `camera_left_offset_m`: camera offset from the robot reference point.
- `min_area_px`: minimum detected marker area used by the navigation loop.
- `target_distance_m` and `target_distance_tolerance_m`: target standoff distance and accepted distance error.
- `heading_kp`, `heading_ki`, `heading_kd`: heading PID gains. Heading control uses radians internally.
- `distance_kp`, `distance_ki`, `distance_kd`: distance PID gains.
- `target_search_delay_s`: how long the target may be missing during tracking before search restarts.
- `search_servo_step_deg` and `search_servo_dwell_s`: servo sweep step and dwell time.
- `servo_center_angle_deg`: servo angle that points the camera straight ahead.
- `search_turn_tolerance_deg`: yaw-error threshold used to finish the in-place turn after search finds the target.
- `show_preview`: enables or disables the OpenCV preview window.

## Arduino Firmware

Open `elegoo/elegoo.ino` in the Arduino IDE and upload it to the Arduino Uno.

Required Arduino libraries:

- `Adafruit MPU6050`
- `Adafruit Unified Sensor`
- `Servo`

Serial protocol at `115200` baud:

- Arduino to Raspberry Pi: `SENSOR <yaw_deg> <distance_cm>`
- Raspberry Pi to Arduino: `MOTOR <left_pwm> <right_pwm>`
- Raspberry Pi to Arduino: `SERVO <angle_deg>`

The Arduino loop runs the yaw update and ultrasonic read, reports the latest sensor line, handles serial commands, and stops the motors if fresh motor commands are not received.

## ESP32 Camera Firmware

Open `esp32/esp32.ino` in the Arduino IDE with ESP32 board support installed. Set the board options:

- Board: `ESP32S3 Dev Module`
- USB CDC On Boot: `Enabled`
- Flash Size: `8MB(64Mb)`
- Partition Scheme: `8M with spiffs (3MB APP/1.5MB SPIFFS)`
- PSRAM: `OPI PSRAM`

Set `WifiSettings::mode` in `esp32/esp32.hpp`:

- `WifiMode::AccessPoint`: the ESP32 creates the configured access point. Connect the Raspberry Pi to that network and set `Settings.stream_url` to `http://192.168.4.1/stream` unless the access point IP is changed.
- `WifiMode::Station`: the ESP32 joins the configured Wi-Fi network. Set `WifiSettings::stationSsid` and `WifiSettings::stationPassword`, upload the firmware, read the ESP32 IP address from serial output, then set `Settings.stream_url` to `http://<esp32-ip>/stream`.

## Run Checklist

1. Upload `elegoo/elegoo.ino` to the Arduino Uno.
2. Upload `esp32/esp32.ino` to the ESP32 camera board.
3. Generate or verify `raspberry/camera_calibration.npz`.
4. Connect the Arduino Uno to the Raspberry Pi over USB.
5. Connect the Raspberry Pi to the ESP32 access point, or put the Raspberry Pi and ESP32 on the same station-mode Wi-Fi network.
6. Update `Settings` in `raspberry/settings.py`.
7. Run `python3 main.py` from the `raspberry/` directory.
