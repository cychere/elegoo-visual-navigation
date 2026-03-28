from __future__ import annotations

import cv2
import sys
import time
import math
import socket
import numpy as np
from typing import Optional
from urllib.error import URLError
from dataclasses import dataclass, field
from urllib.request import Request, urlopen
from arduino_io import ArduinoLink, SensorReading
from motor_mixer import RobotCommand, MixerSettings, WheelCommand, mix_drive_command


@dataclass(slots=True)
class Detection:
    marker_id: int
    corners: "np.ndarray"
    center_x: float
    center_y: float
    area_px: float


@dataclass(slots=True)
class VisualMeasurement:
    angle_deg: float
    marker_id: int
    center_x: float
    center_y: float
    distance_m: Optional[float]


@dataclass(slots=True)
class ControlDecision:
    drive: RobotCommand
    wheels: WheelCommand
    sensor: Optional[SensorReading]
    target_visible: bool
    target_distance_m: Optional[float]


@dataclass(slots=True)
class Settings:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    stream_url: str = "http://192.168.50.48/stream"
    target_marker_id: Optional[int] = None
    target_payload: Optional[str] = None
    aruco_dictionary_name: str = "DICT_4X4_50"
    horizontal_fov_deg: float = 62.2
    marker_size_m: Optional[float] = None
    qr_size_m: Optional[float] = None
    camera_forward_offset_m: float = 0.0
    camera_left_offset_m: float = 0.0
    min_area_px: float = 500.0
    smoothing: float = 0.2
    report_hz: float = 5.0
    reconnect_delay_s: float = 1.0
    stream_timeout_s: float = 5.0
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
    mixer: MixerSettings = field(default_factory=MixerSettings)


class MjpegStream:
    def __init__(self, stream_url: str, timeout_s: float) -> None:
        request = Request(
            stream_url,
            headers={"User-Agent": "elegoo-visual-navigation/1.0"},
        )
        self._response = urlopen(request, timeout=timeout_s)
        self._buffer = bytearray()
        self._chunk_size = 4096
        self._max_buffer_size = 2 * 1024 * 1024

    def read(self) -> "np.ndarray":
        while True:
            start = self._buffer.find(b"\xff\xd8")
            end = self._buffer.find(b"\xff\xd9", start + 2 if start != -1 else 0)

            if start != -1 and end != -1 and end > start:
                jpeg = bytes(self._buffer[start : end + 2])
                del self._buffer[: end + 2]
                frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    return frame

            chunk = self._response.read(self._chunk_size)
            if not chunk:
                raise RuntimeError("Camera stream closed.")

            self._buffer.extend(chunk)
            if len(self._buffer) > self._max_buffer_size:
                last_start = self._buffer.rfind(b"\xff\xd8")
                if last_start == -1:
                    self._buffer.clear()
                else:
                    del self._buffer[:last_start]

    def close(self) -> None:
        self._response.close()


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def wrap_degrees(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


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


def focal_length_px(frame_width: int, horizontal_fov_deg: float) -> float:
    half_fov_rad = math.radians(horizontal_fov_deg / 2.0)
    return (frame_width / 2.0) / math.tan(half_fov_rad)


def build_camera_matrix(
    frame_width: int, frame_height: int, horizontal_fov_deg: float
) -> "np.ndarray":
    focal_px = focal_length_px(frame_width, horizontal_fov_deg)
    cx = (frame_width - 1) / 2.0
    cy = (frame_height - 1) / 2.0
    return np.array(
        [[focal_px, 0.0, cx], [0.0, focal_px, cy], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )


@dataclass(slots=True)
class ArucoDetectorBundle:
    dictionary: object
    parameters: object
    detector: Optional[object]


def build_aruco_detector(dictionary_name: str) -> ArucoDetectorBundle:
    if not hasattr(cv2, "aruco"):
        raise ModuleNotFoundError(
            "ArUco support requires OpenCV's contrib modules. Install `opencv-contrib-python`."
        )

    aruco = cv2.aruco
    try:
        dictionary_id = getattr(aruco, dictionary_name)
    except AttributeError as exc:
        raise ValueError(f"Unknown ArUco dictionary: {dictionary_name}") from exc

    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:
        parameters = aruco.DetectorParameters_create()

    detector = None
    if hasattr(aruco, "ArucoDetector"):
        detector = aruco.ArucoDetector(dictionary, parameters)

    return ArucoDetectorBundle(
        dictionary=dictionary,
        parameters=parameters,
        detector=detector,
    )


def configured_target_marker_id(settings: Settings) -> Optional[int]:
    if settings.target_marker_id is not None:
        return settings.target_marker_id

    if settings.target_payload is None:
        return None

    try:
        return int(settings.target_payload)
    except ValueError:
        return None


def configured_marker_size_m(settings: Settings) -> Optional[float]:
    if settings.marker_size_m is not None:
        return settings.marker_size_m
    return settings.qr_size_m


def detect_aruco_markers(
    detector_bundle: ArucoDetectorBundle, frame: "np.ndarray"
) -> list[Detection]:
    detections: list[Detection] = []

    if detector_bundle.detector is not None:
        corners_list, ids, _rejected = detector_bundle.detector.detectMarkers(frame)
    else:
        corners_list, ids, _rejected = cv2.aruco.detectMarkers(
            frame,
            detector_bundle.dictionary,
            parameters=detector_bundle.parameters,
        )

    if ids is None:
        return detections

    marker_ids = np.asarray(ids, dtype=np.int32).reshape(-1)
    for marker_id, corners in zip(marker_ids, corners_list):
        corner_array = np.asarray(corners, dtype=np.float32).reshape(4, 2)
        center = corner_array.mean(axis=0)
        detections.append(
            Detection(
                marker_id=int(marker_id),
                corners=corner_array,
                center_x=float(center[0]),
                center_y=float(center[1]),
                area_px=abs(float(cv2.contourArea(corner_array))),
            )
        )
    return detections


def select_detection(
    detections: list[Detection], target_marker_id: Optional[int], min_area_px: float
) -> Optional[Detection]:
    filtered = [detection for detection in detections if detection.area_px >= min_area_px]
    if not filtered:
        return None

    if target_marker_id is not None:
        exact_matches = [d for d in filtered if d.marker_id == target_marker_id]
        return max(exact_matches, key=lambda item: item.area_px) if exact_matches else None

    return max(filtered, key=lambda item: item.area_px)


def estimate_marker_pose(
    detection: Detection,
    marker_size_m: float,
    camera_matrix: "np.ndarray",
) -> tuple[Optional[float], Optional[float], Optional[float]]:
    half_size = marker_size_m / 2.0
    object_points = np.array(
        [
            [-half_size, -half_size, 0.0],
            [half_size, -half_size, 0.0],
            [half_size, half_size, 0.0],
            [-half_size, half_size, 0.0],
        ],
        dtype=np.float32,
    )
    distortion = np.zeros((4, 1), dtype=np.float32)

    success, _rvec, tvec = cv2.solvePnP(
        object_points,
        detection.corners.astype(np.float32),
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None, None, None

    tx = float(tvec[0, 0])
    tz = float(tvec[2, 0])

    if tz <= 0.0:
        return None, None, None

    forward_m = tz
    left_m = -tx
    return forward_m, left_m, math.hypot(forward_m, left_m)


def measure_target(
    detection: Detection,
    frame_width: int,
    frame_height: int,
    horizontal_fov_deg: float,
    marker_size_m: Optional[float],
    camera_forward_offset_m: float,
    camera_left_offset_m: float,
) -> VisualMeasurement:
    frame_center_x = (frame_width - 1) / 2.0
    focal_px = focal_length_px(frame_width, horizontal_fov_deg)
    angle_deg = math.degrees(math.atan2(frame_center_x - detection.center_x, focal_px))
    distance_m = None

    if marker_size_m is not None and marker_size_m > 0.0:
        camera_matrix = build_camera_matrix(frame_width, frame_height, horizontal_fov_deg)
        pose_forward_m, pose_left_m, pose_distance_m = estimate_marker_pose(
            detection, marker_size_m, camera_matrix
        )
        if (
            pose_forward_m is not None
            and pose_left_m is not None
            and pose_distance_m is not None
        ):
            robot_forward_m = pose_forward_m + camera_forward_offset_m
            robot_left_m = pose_left_m + camera_left_offset_m
            angle_deg = math.degrees(math.atan2(robot_left_m, robot_forward_m))
            distance_m = math.hypot(robot_forward_m, robot_left_m)

    return VisualMeasurement(
        angle_deg=wrap_degrees(angle_deg),
        marker_id=detection.marker_id,
        center_x=detection.center_x,
        center_y=detection.center_y,
        distance_m=distance_m,
    )


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


def draw_overlay(
    frame: "np.ndarray",
    detection: Optional[Detection],
    measurement: Optional[VisualMeasurement],
    decision: ControlDecision,
    settings: Settings,
) -> "np.ndarray":
    canvas = frame.copy()
    frame_height, frame_width = canvas.shape[:2]
    center_x = int((frame_width - 1) / 2.0)
    cv2.line(canvas, (center_x, 0), (center_x, frame_height - 1), (90, 90, 90), 1)

    if detection is not None:
        polygon = detection.corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [polygon], isClosed=True, color=(0, 220, 0), thickness=2)
        cv2.circle(
            canvas,
            (int(round(detection.center_x)), int(round(detection.center_y))),
            5,
            (0, 220, 0),
            -1,
        )

    lines = [
        f"Stream: {settings.stream_url}",
        f"Target visible: {'yes' if decision.target_visible else 'no'}",
        f"Robot angle: {decision.drive.angle_deg:+6.1f} deg",
        f"Robot speed: {decision.drive.speed:.2f}",
        f"Wheels: L {decision.wheels.left_pwm:+4d} | R {decision.wheels.right_pwm:+4d}",
    ]

    if decision.sensor is not None:
        lines.append(f"Yaw: {decision.sensor.yaw_deg:+7.2f} deg")
        lines.append(f"Ultrasonic: {decision.sensor.distance_cm:4d} cm")
    else:
        lines.append("Yaw: unavailable")
        lines.append("Ultrasonic: unavailable")

    if measurement is not None:
        lines.append(f"Marker angle: {measurement.angle_deg:+6.1f} deg")
        if measurement.distance_m is not None:
            lines.append(f"Marker distance: {measurement.distance_m:.2f} m")
        lines.append(f"Marker ID: {measurement.marker_id}")

    y = 24
    for line in lines:
        cv2.putText(
            canvas,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            line,
            (12, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (20, 20, 20),
            1,
            cv2.LINE_AA,
        )
        y += 24

    return canvas


def emit_report(decision: ControlDecision, measurement: Optional[VisualMeasurement]) -> None:
    parts = [
        f"visible={'yes' if decision.target_visible else 'no'}",
        f"robot_angle_deg={decision.drive.angle_deg:+.2f}",
        f"robot_speed={decision.drive.speed:.3f}",
        f"left_pwm={decision.wheels.left_pwm:+d}",
        f"right_pwm={decision.wheels.right_pwm:+d}",
    ]

    if decision.sensor is not None:
        parts.append(f"yaw_deg={decision.sensor.yaw_deg:+.2f}")
        parts.append(f"distance_cm={decision.sensor.distance_cm}")

    if measurement is not None and measurement.distance_m is not None:
        parts.append(f"marker_distance_m={measurement.distance_m:.3f}")
    if measurement is not None:
        parts.append(f"marker_id={measurement.marker_id}")

    print(" ".join(parts), flush=True)


def open_stream(stream_url: str, timeout_s: float) -> MjpegStream:
    try:
        return MjpegStream(stream_url, timeout_s)
    except (URLError, TimeoutError, socket.timeout, OSError) as exc:
        raise RuntimeError(f"Unable to open camera stream: {stream_url} ({exc})") from exc


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

    wheels = mix_drive_command(drive, settings.mixer)
    return ControlDecision(
        drive=drive,
        wheels=wheels,
        sensor=sensor,
        target_visible=target_visible,
        target_distance_m=measurement.distance_m if measurement is not None else None,
    )


def main() -> int:
    if cv2 is None or np is None:
        missing = []
        if cv2 is None:
            missing.append("opencv-python")
        if np is None:
            missing.append("numpy")
        missing_text = ", ".join(missing)
        raise ModuleNotFoundError(
            f"The navigation loop requires {missing_text}. Install them before running this script."
        )

    settings = Settings()
    detector = build_aruco_detector(settings.aruco_dictionary_name)
    stream: Optional[MjpegStream] = None
    smoothed_angle_deg: Optional[float] = None
    remembered_target_heading_deg: Optional[float] = None
    remembered_at_s = 0.0
    next_report_time = 0.0
    target_marker_id = configured_target_marker_id(settings)
    marker_size_m = configured_marker_size_m(settings)

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

                current_time = time.time()
                if current_time >= next_report_time:
                    emit_report(decision, measurement)
                    next_report_time = current_time + (1.0 / settings.report_hz)

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
