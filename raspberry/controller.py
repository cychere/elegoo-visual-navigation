from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from settings import Settings

if TYPE_CHECKING:
    from arduino_io import SensorReading
    from vision import Detection, VisualMeasurement


MODE_STOP = "stop"
MODE_TRACKING = "tracking"
MODE_SEARCHING = "searching"
MODE_TURNING = "turning"


def wrap_degrees(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


@dataclass(slots=True)
class TargetObservation:
    marker_id: int
    bearing_deg: float


@dataclass(slots=True)
class ControllerUpdate:
    servo_angle_deg: int | None = None
    reset_control: bool = False


@dataclass(slots=True)
class SequentialTargetController:
    settings: Settings
    mode: str = MODE_STOP
    active_target_id: int | None = None
    turn_target_yaw_deg: float | None = None
    lost_target_since_s: float | None = None
    target_index: int = 0
    search_observation: TargetObservation | None = None
    search_angles_deg: tuple[int, ...] = field(init=False)
    search_index: int = 0
    search_step_started_s: float = 0.0

    def __post_init__(self) -> None:
        self.search_angles_deg = tuple(
            list(range(0, 181, self.settings.search_servo_step_deg))
            + ([] if 180 % self.settings.search_servo_step_deg == 0 else [180])
        )

    @property
    def target_ids(self) -> tuple[int, ...]:
        return self.settings.target_marker_ids

    def start(self, now_s: float) -> ControllerUpdate:
        return self._begin_search(now_s)

    def turning_yaw_error_deg(self, sensor: SensorReading) -> float:
        return wrap_degrees(self.turn_target_yaw_deg - sensor.yaw_deg)

    def update(
        self,
        detections: dict[int, Detection],
        measurements: dict[int, VisualMeasurement],
        sensor: SensorReading,
        now_s: float,
    ) -> ControllerUpdate:
        if self.mode == MODE_SEARCHING:
            return self._update_searching(detections, measurements, sensor, now_s)

        if self.mode == MODE_TURNING:
            return self._update_turning(sensor)

        if self.mode == MODE_TRACKING:
            return self._update_tracking(measurements, now_s)

        return ControllerUpdate()

    def _begin_search(self, now_s: float) -> ControllerUpdate:
        self.mode = MODE_SEARCHING
        self.active_target_id = None
        self.turn_target_yaw_deg = None
        self.lost_target_since_s = None
        self.search_observation = None
        self.search_index = 0
        self.search_step_started_s = now_s
        return ControllerUpdate(
            servo_angle_deg=self.search_angles_deg[self.search_index],
            reset_control=True,
        )

    def _record_search_observation(
        self,
        detections: dict[int, Detection],
        measurements: dict[int, VisualMeasurement],
    ) -> None:
        marker_id = self.target_ids[self.target_index]
        measurement = measurements.get(marker_id)
        if measurement is not None and detections.get(marker_id) is not None:
            self.search_observation = TargetObservation(
                marker_id=marker_id,
                bearing_deg=wrap_degrees(
                    self._servo_heading_offset_deg()
                    + measurement.angle_deg
                ),
            )

    def _update_searching(
        self,
        detections: dict[int, Detection],
        measurements: dict[int, VisualMeasurement],
        sensor: SensorReading,
        now_s: float,
    ) -> ControllerUpdate:
        self._record_search_observation(detections, measurements)

        if self.search_observation is not None:
            return self._turn_to_search_observation(sensor, self.search_observation)

        if now_s - self.search_step_started_s < self.settings.search_servo_dwell_s:
            return ControllerUpdate()

        if self.search_index == len(self.search_angles_deg) - 1:
            self.search_index = 0
        else:
            self.search_index += 1

        self.search_step_started_s = now_s
        return ControllerUpdate(servo_angle_deg=self.search_angles_deg[self.search_index])

    def _turn_to_search_observation(
        self,
        sensor: SensorReading,
        observation: TargetObservation,
    ) -> ControllerUpdate:
        self.mode = MODE_TURNING
        self.active_target_id = observation.marker_id
        self.turn_target_yaw_deg = sensor.yaw_deg + observation.bearing_deg
        return ControllerUpdate(
            servo_angle_deg=self._centered_servo_angle_deg(),
            reset_control=True,
        )

    def _update_turning(self, sensor: SensorReading) -> ControllerUpdate:
        yaw_error_deg = self.turning_yaw_error_deg(sensor)
        if abs(yaw_error_deg) <= self.settings.search_turn_tolerance_deg:
            self.mode = MODE_TRACKING
            self.turn_target_yaw_deg = None
            self.lost_target_since_s = None
            return ControllerUpdate(reset_control=True)

        return ControllerUpdate()

    def _update_tracking(
        self,
        measurements: dict[int, VisualMeasurement],
        now_s: float,
    ) -> ControllerUpdate:
        measurement = measurements.get(self.active_target_id)

        if measurement is not None:
            self.lost_target_since_s = None
            if (
                measurement.distance_m is not None
                and abs(measurement.distance_m - self.settings.target_distance_m)
                <= self.settings.target_distance_tolerance_m
            ):
                self.target_index += 1
                if self.target_index == len(self.target_ids):
                    self.mode = MODE_STOP
                    self.active_target_id = None
                    return ControllerUpdate(reset_control=True)

                return self._begin_search(now_s)

            return ControllerUpdate()

        if self.lost_target_since_s is None:
            self.lost_target_since_s = now_s

        if now_s - self.lost_target_since_s >= self.settings.target_search_delay_s:
            return self._begin_search(now_s)

        return ControllerUpdate()

    def _servo_heading_offset_deg(self) -> float:
        return (
            float(self.search_angles_deg[self.search_index])
            - self.settings.servo_center_angle_deg
        )

    def _centered_servo_angle_deg(self) -> int:
        return int(round(self.settings.servo_center_angle_deg))
