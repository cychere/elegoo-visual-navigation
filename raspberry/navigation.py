from __future__ import annotations

import cv2
import time
from typing import Optional
from dataclasses import dataclass
from arduino_io import ArduinoLink, SensorReading
from motor_mixer import RobotCommand, WheelCommand, mix_drive_command
from vision import (
    VisualMeasurement,
    build_aruco_detector,
    detect_aruco_markers,
    draw_overlay,
    load_camera_calibration,
    measure_target,
    open_stream,
    select_detection,
    wrap_degrees,
)


@dataclass(slots=True)
class ControlDecision:
    robot: RobotCommand
    wheels: WheelCommand
    sensor: Optional[SensorReading]
    target_visible: bool
    target_distance_m: Optional[float]


@dataclass(slots=True)
class Settings:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    stream_url: str = "http://192.168.50.48/stream"
    stream_timeout_s: float = 5.0

    target_marker_id: Optional[int] = None
    target_payload: Optional[str] = None
    aruco_dictionary_name: str = "DICT_4X4_50"
    marker_size_m: Optional[float] = 0.05

    camera_calibration_path: str = "camera_calibration.npz"
    camera_forward_offset_m: float = 0.0
    camera_left_offset_m: float = 0.0

    min_area_px: float = 500.0
    smoothing: float = 0.2
    show_preview: bool = True

    target_distance_m: float = 0.45
    distance_gain: float = 1.4
    cruise_speed: float = 0.45
    max_speed: float = 1.0
    stop_distance_cm: int = 18
    caution_distance_cm: int = 45
    turn_in_place_angle_deg: float = 35.0
    max_heading_for_speed_deg: float = 70.0
    target_memory_s: float = 1.5


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def smooth_angle_deg(
    previous_deg: Optional[float], current_deg: float, alpha: float
) -> float:
    if previous_deg is None or alpha <= 0.0:
        return wrap_degrees(current_deg)

    alpha = clamp(alpha, 0.0, 1.0)
    previous_rad = math.radians(previous_deg)
    current_rad = math.radians(current_deg)
    x = (1.0 - alpha) * math.cos(previous_rad) + alpha * math.cos(current_rad)
    y = (1.0 - alpha) * math.sin(previous_rad) + alpha * math.sin(current_rad)
    return math.degrees(math.atan2(y, x))


def configured_target_marker_id(settings: Settings) -> Optional[int]:
    if settings.target_marker_id is not None:
        return settings.target_marker_id

    if settings.target_payload is None:
        return None

    return int(settings.target_payload)


def compute_decision(
    measurement: Optional[VisualMeasurement],
    sensor: Optional[SensorReading],
    remembered_target_heading_deg: Optional[float],
    remembered_at_s: float,
    now_s: float,
    settings: Settings,
) -> ControlDecision:
    drive = RobotCommand(angle_deg=0.0, speed=0.0)
    target_visible = measurement is not None

    if measurement is not None:
        drive = RobotCommand(
            angle_deg=measurement.angle_deg,
            speed=speed_from_target(measurement, sensor, settings),
        )
    elif (
        sensor is not None
        and remembered_target_heading_deg is not None
        and now_s - remembered_at_s <= settings.target_memory_s
    ):
        drive = RobotCommand(
            angle_deg=wrap_degrees(remembered_target_heading_deg - sensor.yaw_deg),
            speed=0.0,
        )

    wheels = mix_drive_command(drive)
    return ControlDecision(
        robot=drive,
        wheels=wheels,
        sensor=sensor,
        target_visible=target_visible,
        target_distance_m=measurement.distance_m if measurement is not None else None,
    )


def main() -> int:
    settings = Settings()
    camera_calibration = load_camera_calibration(settings.camera_calibration_path)
    detector = build_aruco_detector(settings.aruco_dictionary_name)
    heading_pid = PIDController(
        kp=settings.heading_kp,
        ki=settings.heading_ki,
        kd=settings.heading_kd,
    )
    distance_pid = PIDController(
        kp=settings.distance_kp,
        ki=settings.distance_ki,
        kd=settings.distance_kd,
    )
    target_marker_id = configured_target_marker_id(settings)
    marker_size_m = settings.marker_size_m

    with ArduinoLink(port=settings.serial_port, baud_rate=settings.baud_rate) as arduino:
        latest_sensor = arduino.wait_for_reading()
        stream = open_stream(settings.stream_url, settings.stream_timeout_s)

        while True:
            latest_sensor = arduino.read_latest() or latest_sensor
            frame = stream.read()
            frame_height, frame_width = frame.shape[:2]
            detections = detect_aruco_markers(detector, frame)
            target = select_detection(
                detections,
                target_marker_id,
                settings.min_area_px,
            )

            measurement: Optional[VisualMeasurement] = None
            if target is not None:
                measurement = measure_target(
                    detection=target,
                    frame_width=frame_width,
                    frame_height=frame_height,
                    camera_calibration=camera_calibration,
                    marker_size_m=settings.marker_size_m,
                    camera_forward_offset_m=settings.camera_forward_offset_m,
                    camera_left_offset_m=settings.camera_left_offset_m,
                )

            now_s = time.monotonic()
            decision = compute_decision(
                measurement=measurement,
                sensor=latest_sensor,
                now_s=now_s,
                heading_pid=heading_pid,
                distance_pid=distance_pid,
                settings=settings,
            )
            arduino.send_motor(decision.wheels.left_pwm, decision.wheels.right_pwm)

            if not settings.show_preview:
                continue

            preview = draw_overlay(
                frame=frame,
                detection=target,
                measurement=measurement,
                decision=decision,
                settings=settings,
            )
            cv2.imshow("Raspberry Navigation", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break

        arduino.stop()
        stream.close()
        if settings.show_preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
