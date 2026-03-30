from __future__ import annotations

import cv2
import sys
import time
import math
import socket
from typing import Optional
from urllib.error import URLError
from dataclasses import dataclass
from arduino_io import ArduinoLink, SensorReading
from motor_mixer import RobotCommand, WheelCommand, mix_drive_command
from vision import (
    MjpegStream,
    VisualMeasurement,
    build_aruco_detector,
    detect_aruco_markers,
    draw_overlay,
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
    reconnect_delay_s: float = 1.0
    stream_timeout_s: float = 5.0

    target_marker_id: Optional[int] = None
    target_payload: Optional[str] = None
    aruco_dictionary_name: str = "DICT_4X4_50"
    marker_size_m: Optional[float] = None

    horizontal_fov_deg: float = 62.2
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

    try:
        return int(settings.target_payload)
    except ValueError:
        return None


def obstacle_speed_scale(distance_cm: int, settings: Settings) -> float:
    if distance_cm <= settings.stop_distance_cm:
        return 0.0
    if distance_cm >= settings.caution_distance_cm:
        return 1.0

    span = settings.caution_distance_cm - settings.stop_distance_cm
    return clamp((distance_cm - settings.stop_distance_cm) / span, 0.0, 1.0)


def speed_from_target(
    measurement: VisualMeasurement,
    sensor: Optional[SensorReading],
    settings: Settings,
) -> float:
    if sensor is not None:
        obstacle_scale = obstacle_speed_scale(sensor.distance_cm, settings)
        if obstacle_scale <= 0.0:
            return 0.0
    else:
        obstacle_scale = 1.0

    if abs(measurement.angle_deg) >= settings.turn_in_place_angle_deg:
        return 0.0

    if measurement.distance_m is not None:
        distance_error = measurement.distance_m - settings.target_distance_m
        requested_speed = clamp(distance_error * settings.distance_gain, 0.0, settings.max_speed)
    else:
        requested_speed = settings.cruise_speed

    heading_scale = clamp(
        1.0 - (abs(measurement.angle_deg) / settings.max_heading_for_speed_deg),
        0.0,
        1.0,
    )
    return clamp(requested_speed * obstacle_scale * heading_scale, 0.0, settings.max_speed)


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
    detector = build_aruco_detector(settings.aruco_dictionary_name)
    stream: Optional[MjpegStream] = None
    smoothed_angle_deg: Optional[float] = None
    remembered_target_heading_deg: Optional[float] = None
    remembered_at_s = 0.0
    target_marker_id = configured_target_marker_id(settings)
    marker_size_m = settings.marker_size_m

    with ArduinoLink(port=settings.serial_port, baud_rate=settings.baud_rate) as arduino:
        latest_sensor = arduino.wait_for_reading()

        try:
            while True:
                latest_sensor = arduino.read_latest() or latest_sensor

                if stream is None:
                    try:
                        stream = open_stream(settings.stream_url, settings.stream_timeout_s)
                    except RuntimeError as exc:
                        print(str(exc), file=sys.stderr)
                        arduino.stop()
                        time.sleep(settings.reconnect_delay_s)
                        continue

                try:
                    frame = stream.read()
                except RuntimeError as exc:
                    print(str(exc), file=sys.stderr)
                    stream.close()
                    stream = None
                    arduino.stop()
                    time.sleep(settings.reconnect_delay_s)
                    continue
                except (URLError, TimeoutError, socket.timeout, OSError) as exc:
                    print(f"Camera stream read failed: {exc}", file=sys.stderr)
                    stream.close()
                    stream = None
                    arduino.stop()
                    time.sleep(settings.reconnect_delay_s)
                    continue

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
                        horizontal_fov_deg=settings.horizontal_fov_deg,
                        marker_size_m=marker_size_m,
                        camera_forward_offset_m=settings.camera_forward_offset_m,
                        camera_left_offset_m=settings.camera_left_offset_m,
                    )
                    measurement.angle_deg = smooth_angle_deg(
                        smoothed_angle_deg,
                        measurement.angle_deg,
                        settings.smoothing,
                    )
                    smoothed_angle_deg = measurement.angle_deg

                    if latest_sensor is not None:
                        remembered_target_heading_deg = wrap_degrees(
                            latest_sensor.yaw_deg + measurement.angle_deg
                        )
                        remembered_at_s = time.monotonic()
                else:
                    smoothed_angle_deg = None

                now_s = time.monotonic()
                decision = compute_decision(
                    measurement=measurement,
                    sensor=latest_sensor,
                    remembered_target_heading_deg=remembered_target_heading_deg,
                    remembered_at_s=remembered_at_s,
                    now_s=now_s,
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
        finally:
            arduino.stop()
            if stream is not None:
                stream.close()
            if settings.show_preview:
                cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
