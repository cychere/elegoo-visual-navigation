from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from angles import wrap_degrees
from settings import Settings

if TYPE_CHECKING:
    from arduino_io import SensorReading
    from vision import Detection, VisualMeasurement


MODE_STOP = "stop"
MODE_TRACKING = "tracking"
MODE_SEARCHING = "searching"
MODE_TURNING = "turning"


@dataclass(slots=True)
class TargetObservation:
    marker_id: int
    bearing_deg: float
    distance_m: float | None
    area_px: float

    def score(self) -> tuple[float, float, float]:
        distance = self.distance_m if self.distance_m is not None else float("inf")
        return distance, abs(self.bearing_deg), -self.area_px


@dataclass(slots=True)
class ControllerUpdate:
    servo_angle_deg: int | None = None
    reset_control: bool = False


@dataclass(slots=True)
class RecedingHorizonController:
    settings: Settings
    mode: str = MODE_STOP
    active_target_id: int | None = None
    turn_target_yaw_deg: float | None = None
    lost_target_since_s: float | None = None
    visited_target_ids: set[int] = field(default_factory=set)
    search_observations: dict[int, TargetObservation] = field(default_factory=dict)
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
        return self.settings.target_marker_ids[: self.settings.number_of_targets]

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

    def _remaining_target_ids(self) -> tuple[int, ...]:
        return tuple(
            marker_id
            for marker_id in self.target_ids
            if marker_id not in self.visited_target_ids
        )

    def _begin_search(self, now_s: float) -> ControllerUpdate:
        self.mode = MODE_SEARCHING
        self.active_target_id = None
        self.turn_target_yaw_deg = None
        self.lost_target_since_s = None
        self.search_observations.clear()
        self.search_index = 0
        self.search_step_started_s = now_s
        return ControllerUpdate(
            servo_angle_deg=self.search_angles_deg[self.search_index],
            reset_control=True,
        )

    def _record_search_observations(
        self,
        detections: dict[int, Detection],
        measurements: dict[int, VisualMeasurement],
    ) -> None:
        for marker_id in self._remaining_target_ids():
            measurement = measurements.get(marker_id)
            detection = detections.get(marker_id)
            if measurement is not None and detection is not None:
                self.search_observations[marker_id] = TargetObservation(
                    marker_id=marker_id,
                    bearing_deg=wrap_degrees(
                        self._servo_heading_offset_deg()
                        + measurement.angle_deg
                    ),
                    distance_m=measurement.distance_m,
                    area_px=detection.area_px,
                )

    def _update_searching(
        self,
        detections: dict[int, Detection],
        measurements: dict[int, VisualMeasurement],
        sensor: SensorReading,
        now_s: float,
    ) -> ControllerUpdate:
        self._record_search_observations(detections, measurements)

        if self.search_observations.keys() >= set(self._remaining_target_ids()):
            return self._choose_next_target(sensor)

        if now_s - self.search_step_started_s < self.settings.search_servo_dwell_s:
            return ControllerUpdate()

        if self.search_index == len(self.search_angles_deg) - 1:
            if self.search_observations:
                return self._choose_next_target(sensor)

            self.search_index = 0
        else:
            self.search_index += 1

        self.search_step_started_s = now_s
        return ControllerUpdate(servo_angle_deg=self.search_angles_deg[self.search_index])

    def _choose_next_target(self, sensor: SensorReading) -> ControllerUpdate:
        observation = min(
            self.search_observations.values(),
            key=lambda candidate: candidate.score(),
        )
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
                self.visited_target_ids.add(measurement.marker_id)
                if len(self.visited_target_ids) == len(self.target_ids):
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
