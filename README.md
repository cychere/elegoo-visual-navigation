# elegoo-visual-navigation

Visual navigation software for an ELEGOO Smart Robot Car. The robot uses an ESP32 camera stream to detect an ArUco target marker, estimates the target bearing and distance on a Raspberry Pi, then sends wheel PWM commands to an Arduino Uno that drives the motors and reads onboard sensors.

When the target disappears, the robot waits for a configurable delay, sweeps the camera servo in fixed increments to reacquire the marker, then recenters the camera and rotates in place to the found heading before resuming normal navigation.

Simplified Chinese version: [README.zh-CN.md](README.zh-CN.md)

## System

- ELEGOO Smart Robot Car chassis with four-wheel drive
- Arduino Uno for motors, servo, ultrasonic sensor, and MPU6050 yaw integration
- ESP32 camera module mounted on a servo
- Raspberry Pi for vision and navigation
- Printed ArUco target marker

Data flow:

1. The ESP32 hosts an MJPEG camera stream at `http://<esp32-ip>/stream`.
2. The Raspberry Pi reads the stream, detects the target marker, and computes steering and speed.
3. The Raspberry Pi sends `MOTOR <left_pwm> <right_pwm>` commands to the Arduino over serial.
4. The Arduino prints `SENSOR <yaw_deg> <distance_cm>` readings and applies motor or servo commands.

## Repository Layout

- `elegoo/`: Arduino Uno firmware. `elegoo.ino` owns the main loop, serial protocol, motor timeout, ultrasonic reads, servo commands, and gyro yaw tracking.
- `esp32/`: ESP32 camera firmware. `esp32.ino` starts Wi-Fi and the camera server; `app_httpd.cpp` serves the live stream.
- `raspberry/`: Raspberry Pi software. `navigation.py` is the main loop, `vision.py` handles the stream and ArUco detection, `motor_mixer.py` maps robot commands to wheel PWM, and `arduino_io.py` owns the serial link.
- `test/`: Standalone test sketches and scripts.

## Raspberry Pi Setup

Install the Python dependencies on the Raspberry Pi:

```bash
cd raspberry
python3 -m venv .venv
source .venv/bin/activate
pip install numpy pyserial opencv-contrib-python
```

Run navigation from the `raspberry/` directory so `navigation.py` can load `camera_calibration.npz`:

```bash
python3 navigation.py
```

Press `q` or `Esc` in the preview window to stop.

## Camera Calibration

`navigation.py` expects `raspberry/camera_calibration.npz`. Generate it with the calibration script using photos of a 9x6 inner-corner chessboard:

```bash
cd raspberry
python3 camera_calibrate.py "calibration/*.jpg" --square-size-mm 23
```

Use photos captured at the same camera resolution, focus, and lens setting used during navigation. The default output path is `raspberry/camera_calibration.npz`.

## Configuration

Edit the `Settings` dataclass in `raspberry/navigation.py` before running:

- `serial_port`: Arduino serial device, default `/dev/ttyUSB0`
- `stream_url`: ESP32 MJPEG stream URL
- `target_marker_id`: marker ID to track; `None` selects the largest valid marker
- `aruco_dictionary_name`: default `DICT_4X4_50`
- `marker_size_m`: printed marker side length in meters, default `0.05`
- `target_distance_m`: stopping distance from the marker, default `0.45`
- PID values: `heading_kp`, `heading_ki`, `heading_kd`, `distance_kp`, `distance_ki`, `distance_kd`. Heading PID uses radians; preview and serial angles stay in degrees.
- `target_search_delay_s`: how long the target must stay missing before servo search starts
- `search_servo_step_deg`: servo search increment in degrees
- `search_servo_dwell_s`: how long to hold each search angle before stepping again
- `servo_forward_angle_deg`: servo angle that points the camera straight ahead. Default `72.0`, so `0` is right-forward and `180` is left-backward.
- `search_turn_tolerance_deg`: yaw error threshold used to finish the in-place alignment after the target is found during search
- `show_preview`: OpenCV preview window

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

## ESP32 Camera Firmware

Open `esp32/esp32.ino` in the Arduino IDE. Set the board options listed in `esp32/notes.txt`:

- Board: `ESP32S3 Dev Module`
- USB CDC On Boot: `Enabled`
- Flash Size: `8MB(64Mb)`
- Partition Scheme: `8M with spiffs (3MB APP/1.5MB SPIFFS)`
- PSRAM: `OPI PSRAM`

Set the Wi-Fi SSID and password in `esp32/esp32.hpp`, upload the firmware, find the ESP32 IP address on the Wi-Fi network, and set `Settings.stream_url` in `raspberry/navigation.py` to `http://<esp32-ip>/stream`.

## Run Checklist

1. Upload `elegoo/elegoo.ino` to the Arduino Uno.
2. Upload `esp32/esp32.ino` to the ESP32 camera board.
3. Generate or copy `camera_calibration.npz` into `raspberry/`.
4. Connect the Arduino Uno to the Raspberry Pi over USB.
5. Put the Raspberry Pi and ESP32 on the same Wi-Fi network.
6. Update `Settings` in `raspberry/navigation.py`.
7. Run `python3 navigation.py` from the `raspberry/` directory.
