from __future__ import annotations

import cv2
import math
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
    mode: str


@dataclass(slots=True)
class Settings:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    stream_url: str = "http://192.168.4.1/stream"
    stream_timeout_s: float = 5.0

    target_marker_id: Optional[int] = None
    target_payload: Optional[str] = None
    aruco_dictionary_name: str = "DICT_4X4_50"
    marker_size_m: Optional[float] = 0.05

    camera_calibration_path: str = "camera_calibration.npz"
    camera_forward_offset_m: float = 0.0
    camera_left_offset_m: float = 0.0

    min_area_px: float = 500.0
    show_preview: bool = True

    target_distance_m: float = 0.45
    heading_kp: float = 1.0
    heading_ki: float = 0.0
    heading_kd: float = 0.05
    distance_kp: float = 1.4
    distance_ki: float = 0.0
    distance_kd: float = 0.05

    target_search_delay_s: float = 1.5
    search_servo_step_deg: int = 30
    search_servo_dwell_s: float = 1.0
    servo_center_angle_deg: float = 72.0
    search_turn_tolerance_deg: float = 5.0


@dataclass(slots=True)
class PIDController:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    previous_error: Optional[float] = None
    previous_time_s: Optional[float] = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.previous_time_s = None

    def update(self, error: float, now_s: float) -> float:
        derivative = 0.0

        if self.previous_time_s is not None and self.previous_error is not None:
            dt_s = now_s - self.previous_time_s
            self.integral += error * dt_s
            derivative = (error - self.previous_error) / dt_s

        self.previous_error = error
        self.previous_time_s = now_s
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)


def configured_target_marker_id(settings: Settings) -> Optional[int]:
    if settings.target_marker_id is not None:
        return settings.target_marker_id

    if settings.target_payload is None:
        return None

    return int(settings.target_payload)


def centered_servo_angle_deg(settings: Settings) -> int:
    return int(round(settings.servo_center_angle_deg))


def servo_heading_offset_deg(servo_angle_deg: int, settings: Settings) -> float:
    return float(servo_angle_deg) - settings.servo_center_angle_deg


def advance_search_servo(servo_angle_deg: int, direction: int, settings: Settings) -> tuple[int, int]:
    next_angle_deg = servo_angle_deg + (direction * settings.search_servo_step_deg)

    if next_angle_deg <= 0:
        return 0, 1

    if next_angle_deg >= 180:
        return 180, -1

    return next_angle_deg, direction


def stationary_decision(
    sensor: Optional[SensorReading],
    measurement: Optional[VisualMeasurement],
    mode: str,
) -> ControlDecision:
    robot = RobotCommand(turn_effort=0.0, speed_effort=0.0)
    wheels = mix_drive_command(robot)
    return ControlDecision(
        robot=robot,
        wheels=wheels,
        sensor=sensor,
        target_visible=measurement is not None,
        mode=mode,
    )


def turning_decision(
    yaw_error_deg: float,
    sensor: Optional[SensorReading],
    measurement: Optional[VisualMeasurement],
    now_s: float,
    heading_pid: PIDController,
) -> ControlDecision:
    robot = RobotCommand(
        turn_effort=heading_pid.update(math.radians(yaw_error_deg), now_s),
        speed_effort=0.0,
    )
    wheels = mix_drive_command(robot)
    return ControlDecision(
        robot=robot,
        wheels=wheels,
        sensor=sensor,
        target_visible=measurement is not None,
        mode="turning",
    )


def compute_decision(
    measurement: Optional[VisualMeasurement],
    sensor: Optional[SensorReading],
    now_s: float,
    heading_pid: PIDController,
    distance_pid: PIDController,
    settings: Settings,
) -> ControlDecision:
    robot = RobotCommand(turn_effort=0.0, speed_effort=0.0)

    if measurement is not None:
        if measurement.distance_m is not None:
            distance_error = measurement.distance_m - settings.target_distance_m
            speed_effort = distance_pid.update(distance_error, now_s)
        else:
            speed_effort = 0.0
            distance_pid.reset()

        robot = RobotCommand(
            turn_effort=heading_pid.update(measurement.angle_rad, now_s),
            speed_effort=speed_effort,
        )
    else:
        heading_pid.reset()
        distance_pid.reset()

    wheels = mix_drive_command(robot)
    return ControlDecision(
        robot=robot,
        wheels=wheels,
        sensor=sensor,
        target_visible=measurement is not None,
        mode="tracking",
    )


def main() -> int:
    settings = Settings()
    servo_center_angle_deg = centered_servo_angle_deg(settings)
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

    with ArduinoLink(port=settings.serial_port, baud_rate=settings.baud_rate) as arduino:
        latest_sensor = arduino.wait_for_reading()
        arduino.send_servo(servo_center_angle_deg)
        stream = open_stream(settings.stream_url, settings.stream_timeout_s)
        mode = "tracking"
        lost_target_since_s: Optional[float] = None
        search_servo_angle_deg = servo_center_angle_deg
        search_direction = -1
        search_step_started_s = time.monotonic()
        turn_target_yaw_deg: Optional[float] = None

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
            decision = stationary_decision(latest_sensor, measurement, mode)

            if mode == "tracking":
                if measurement is not None:
                    lost_target_since_s = None
                    decision = compute_decision(
                        measurement=measurement,
                        sensor=latest_sensor,
                        now_s=now_s,
                        heading_pid=heading_pid,
                        distance_pid=distance_pid,
                        settings=settings,
                    )
                else:
                    if lost_target_since_s is None:
                        lost_target_since_s = now_s

                    if now_s - lost_target_since_s >= settings.target_search_delay_s:
                        mode = "searching"
                        heading_pid.reset()
                        distance_pid.reset()
                        search_servo_angle_deg, search_direction = advance_search_servo(
                            servo_center_angle_deg,
                            -1,
                            settings,
                        )
                        search_step_started_s = now_s
                        arduino.send_servo(search_servo_angle_deg)
                        decision = stationary_decision(latest_sensor, measurement, mode)

            elif mode == "searching":
                if measurement is not None:
                    target_heading_deg = wrap_degrees(
                        servo_heading_offset_deg(search_servo_angle_deg, settings)
                        + measurement.angle_deg
                    )
                    mode = "turning"
                    heading_pid.reset()
                    distance_pid.reset()
                    arduino.send_servo(servo_center_angle_deg)
                    search_servo_angle_deg = servo_center_angle_deg
                    turn_target_yaw_deg = latest_sensor.yaw_deg + target_heading_deg
                    decision = stationary_decision(latest_sensor, measurement, mode)
                else:
                    if now_s - search_step_started_s >= settings.search_servo_dwell_s:
                        search_servo_angle_deg, search_direction = advance_search_servo(
                            search_servo_angle_deg,
                            search_direction,
                            settings,
                        )
                        search_step_started_s = now_s
                        arduino.send_servo(search_servo_angle_deg)

            elif mode == "turning":
                yaw_error_deg = wrap_degrees(turn_target_yaw_deg - latest_sensor.yaw_deg)
                if abs(yaw_error_deg) <= settings.search_turn_tolerance_deg:
                    mode = "tracking"
                    turn_target_yaw_deg = None
                    heading_pid.reset()
                    distance_pid.reset()
                    lost_target_since_s = None if measurement is not None else now_s
                    decision = stationary_decision(latest_sensor, measurement, mode)
                else:
                    decision = turning_decision(
                        yaw_error_deg=yaw_error_deg,
                        sensor=latest_sensor,
                        measurement=measurement,
                        now_s=now_s,
                        heading_pid=heading_pid,
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
