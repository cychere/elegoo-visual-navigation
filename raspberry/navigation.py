from __future__ import annotations

import cv2
import math
import time
from dataclasses import dataclass

from arduino_io import ArduinoLink, SensorReading
from controller import (
    MODE_SEARCHING,
    MODE_STOP,
    MODE_TRACKING,
    MODE_TURNING,
    ControllerUpdate,
    RecedingHorizonController,
)
from motor_mixer import RobotCommand, WheelCommand, mix_drive_command
from pid import PIDController
from settings import Settings
from vision import (
    Detection,
    VisualMeasurement,
    build_aruco_detector,
    detect_aruco_markers,
    draw_overlay,
    load_camera_calibration,
    measure_target,
    open_stream,
)


@dataclass(slots=True)
class ControlDecision:
    robot: RobotCommand
    wheels: WheelCommand
    sensor: SensorReading | None
    target_visible: bool
    mode: str


def stationary_decision(
    sensor: SensorReading | None,
    measurement: VisualMeasurement | None,
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
    sensor: SensorReading,
    measurement: VisualMeasurement | None,
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
        mode=MODE_TURNING,
    )


def tracking_decision(
    measurement: VisualMeasurement | None,
    sensor: SensorReading,
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
        mode=MODE_TRACKING,
    )


class NavigationRunner:
    def __init__(self, settings: Settings) -> None:
        self.settings = settings
        self.camera_calibration = load_camera_calibration(settings.camera_calibration_path)
        self.detector = build_aruco_detector(settings.aruco_dictionary_name)
        self.heading_pid = PIDController(
            kp=settings.heading_kp,
            ki=settings.heading_ki,
            kd=settings.heading_kd,
        )
        self.distance_pid = PIDController(
            kp=settings.distance_kp,
            ki=settings.distance_ki,
            kd=settings.distance_kd,
        )
        self.controller = RecedingHorizonController(settings)

    def run(self) -> int:
        with ArduinoLink(
            port=self.settings.serial_port,
            baud_rate=self.settings.baud_rate,
        ) as arduino:
            latest_sensor = arduino.wait_for_reading()
            stream = open_stream(self.settings.stream_url, self.settings.stream_timeout_s)
            start_update = self.controller.start(time.monotonic())
            self._apply_update(start_update, arduino)

            while self.controller.mode != MODE_STOP:
                latest_sensor = arduino.read_latest() or latest_sensor
                frame = stream.read()
                frame_height, frame_width = frame.shape[:2]
                detections = {
                    detection.marker_id: detection
                    for detection in detect_aruco_markers(self.detector, frame)
                    if detection.marker_id in self.controller.target_ids
                    and detection.area_px >= self.settings.min_area_px
                }
                measurements = self._measure_detections(
                    detections=detections,
                    frame_width=frame_width,
                    frame_height=frame_height,
                )

                now_s = time.monotonic()
                update = self.controller.update(
                    detections=detections,
                    measurements=measurements,
                    sensor=latest_sensor,
                    now_s=now_s,
                )
                self._apply_update(update, arduino)

                measurement = self._active_measurement(measurements)
                decision = self._make_decision(latest_sensor, measurement, now_s)
                arduino.send_motor(decision.wheels.left_pwm, decision.wheels.right_pwm)

                if self.settings.show_preview:
                    if not self._show_preview(frame, detections, measurement, decision):
                        break

            arduino.stop()
            stream.close()
            if self.settings.show_preview:
                cv2.destroyAllWindows()

        return 0

    def _measure_detections(
        self,
        detections: dict[int, Detection],
        frame_width: int,
        frame_height: int,
    ) -> dict[int, VisualMeasurement]:
        return {
            marker_id: measure_target(
                detection=detection,
                frame_width=frame_width,
                frame_height=frame_height,
                camera_calibration=self.camera_calibration,
                marker_size_m=self.settings.marker_size_m,
                camera_forward_offset_m=self.settings.camera_forward_offset_m,
                camera_left_offset_m=self.settings.camera_left_offset_m,
            )
            for marker_id, detection in detections.items()
        }

    def _apply_update(self, update: ControllerUpdate, arduino: ArduinoLink) -> None:
        if update.reset_control:
            self.heading_pid.reset()
            self.distance_pid.reset()

        if update.servo_angle_deg is not None:
            arduino.send_servo(update.servo_angle_deg)

    def _active_measurement(
        self,
        measurements: dict[int, VisualMeasurement],
    ) -> VisualMeasurement | None:
        if self.controller.active_target_id is None:
            return None

        return measurements.get(self.controller.active_target_id)

    def _make_decision(
        self,
        sensor: SensorReading,
        measurement: VisualMeasurement | None,
        now_s: float,
    ) -> ControlDecision:
        if self.controller.mode == MODE_TRACKING:
            return tracking_decision(
                measurement=measurement,
                sensor=sensor,
                now_s=now_s,
                heading_pid=self.heading_pid,
                distance_pid=self.distance_pid,
                settings=self.settings,
            )

        if self.controller.mode == MODE_TURNING:
            return turning_decision(
                yaw_error_deg=self.controller.turning_yaw_error_deg(sensor),
                sensor=sensor,
                measurement=measurement,
                now_s=now_s,
                heading_pid=self.heading_pid,
            )

        if self.controller.mode in (MODE_SEARCHING, MODE_STOP):
            return stationary_decision(sensor, measurement, self.controller.mode)

    def _show_preview(
        self,
        frame,
        detections: dict[int, Detection],
        measurement: VisualMeasurement | None,
        decision: ControlDecision,
    ) -> bool:
        detection = None
        if self.controller.active_target_id is not None:
            detection = detections.get(self.controller.active_target_id)

        preview = draw_overlay(
            frame=frame,
            detection=detection,
            measurement=measurement,
            decision=decision,
            settings=self.settings,
        )
        cv2.imshow("Raspberry Navigation", preview)
        key = cv2.waitKey(1) & 0xFF
        return key not in (27, ord("q"))


def run(settings: Settings | None = None) -> int:
    runner = NavigationRunner(settings or Settings())
    return runner.run()
