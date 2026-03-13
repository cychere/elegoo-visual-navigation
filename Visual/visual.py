from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class Detection:
    payload: str
    corners: "np.ndarray"
    center_x: float
    center_y: float
    area_px: float


@dataclass
class Measurement:
    angle_deg_left_positive: float
    camera_bearing_deg_left_positive: float
    camera_yaw_deg_left_positive: float
    mode: str
    qr_payload: str
    center_x: float
    center_y: float
    distance_m: Optional[float]
    forward_m: Optional[float]
    left_m: Optional[float]


@dataclass
class Settings:
    stream_url: str = "http://192.168.1.42/stream"
    target_payload: Optional[str] = None
    servo_angle_deg: float = 90.0
    servo_angle_file: Optional[Path] = None
    servo_center_deg: float = 90.0
    servo_positive: str = "left"
    output_positive: str = "left"
    horizontal_fov_deg: float = 62.2
    qr_size_m: Optional[float] = None
    camera_forward_offset_m: float = 0.0
    camera_left_offset_m: float = 0.0
    min_area_px: float = 500.0
    smoothing: float = 0.2
    report_hz: float = 5.0
    reconnect_delay_s: float = 1.0
    show_preview: bool = True
    json_output: bool = False
    servo_step_deg: float = 2.0


# Edit these values directly instead of passing command-line arguments.
SETTINGS = Settings(
    stream_url="http://192.168.50.48/stream",
    target_payload=None,
    servo_angle_deg=90.0,
    servo_angle_file=None,  # Example: Path("Visual/servo_angle.txt")
    servo_center_deg=90.0,
    servo_positive="left",  # "left" or "right"
    output_positive="left",  # "left" or "right"
    horizontal_fov_deg=62.2,
    qr_size_m=None,  # Example: 0.05 for a 5 cm QR code
    camera_forward_offset_m=0.0,
    camera_left_offset_m=0.0,
    min_area_px=500.0,
    smoothing=0.2,
    report_hz=5.0,
    reconnect_delay_s=1.0,
    show_preview=True,
    json_output=False,
    servo_step_deg=2.0,
)


class ServoAngleSource:
    def __init__(self, fixed_angle_deg: float, path: Optional[Path]) -> None:
        self._fixed_angle_deg = fixed_angle_deg
        self._path = path
        self._cached_angle_deg = fixed_angle_deg
        self._cached_mtime_ns: Optional[int] = None

    def read(self) -> float:
        if self._path is None:
            return self._fixed_angle_deg

        try:
            stat = self._path.stat()
        except FileNotFoundError:
            return self._cached_angle_deg

        if self._cached_mtime_ns != stat.st_mtime_ns:
            text = self._path.read_text(encoding="utf-8").strip()
            if text:
                self._cached_angle_deg = clamp(float(text), 0.0, 180.0)
            self._cached_mtime_ns = stat.st_mtime_ns

        return self._cached_angle_deg

    def nudge(self, delta_deg: float) -> float:
        if self._path is not None:
            return self._cached_angle_deg
        self._fixed_angle_deg = clamp(self._fixed_angle_deg + delta_deg, 0.0, 180.0)
        self._cached_angle_deg = self._fixed_angle_deg
        return self._cached_angle_deg


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


def servo_yaw_left_positive_deg(
    servo_angle_deg: float, servo_center_deg: float, servo_positive_left: bool
) -> float:
    sign = 1.0 if servo_positive_left else -1.0
    return (servo_angle_deg - servo_center_deg) * sign


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
        [[focal_px, 0.0, cx], [0.0, focal_px, cy], [0.0, 0.0, 1.0]], dtype=np.float32
    )


def detect_qr_codes(detector: "cv2.QRCodeDetector", frame: "np.ndarray") -> list[Detection]:
    detections: list[Detection] = []

    try:
        ok, decoded_info, points, _ = detector.detectAndDecodeMulti(frame)
    except (AttributeError, ValueError, cv2.error):
        ok, decoded_info, points = False, [], None

    if ok and points is not None:
        for index, corners in enumerate(points):
            corner_array = np.asarray(corners, dtype=np.float32).reshape(4, 2)
            center = corner_array.mean(axis=0)
            detections.append(
                Detection(
                    payload=decoded_info[index] if index < len(decoded_info) else "",
                    corners=corner_array,
                    center_x=float(center[0]),
                    center_y=float(center[1]),
                    area_px=abs(float(cv2.contourArea(corner_array))),
                )
            )

    if detections:
        return detections

    try:
        payload, points, _ = detector.detectAndDecode(frame)
    except cv2.error:
        payload, points = "", None

    if points is None:
        return detections

    corner_array = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = corner_array.mean(axis=0)
    detections.append(
        Detection(
            payload=payload,
            corners=corner_array,
            center_x=float(center[0]),
            center_y=float(center[1]),
            area_px=abs(float(cv2.contourArea(corner_array))),
        )
    )
    return detections


def select_detection(
    detections: list[Detection], target_payload: Optional[str], min_area_px: float
) -> Optional[Detection]:
    filtered = [d for d in detections if d.area_px >= min_area_px]
    if not filtered:
        return None

    if target_payload:
        exact_matches = [d for d in filtered if d.payload == target_payload]
        if exact_matches:
            return max(exact_matches, key=lambda item: item.area_px)

        contains_matches = [d for d in filtered if target_payload in d.payload]
        if contains_matches:
            return max(contains_matches, key=lambda item: item.area_px)

        return None

    return max(filtered, key=lambda item: item.area_px)


def estimate_target_pose(
    detection: Detection,
    qr_size_m: float,
    camera_matrix: "np.ndarray",
) -> tuple[Optional[float], Optional[float], Optional[float]]:
    half_size = qr_size_m / 2.0
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
    servo_angle_deg: float,
    servo_center_deg: float,
    servo_positive_left: bool,
    output_positive_left: bool,
    qr_size_m: Optional[float],
    camera_forward_offset_m: float,
    camera_left_offset_m: float,
) -> Measurement:
    frame_center_x = (frame_width - 1) / 2.0
    focal_px = focal_length_px(frame_width, horizontal_fov_deg)
    camera_bearing_deg = math.degrees(
        math.atan2(frame_center_x - detection.center_x, focal_px)
    )
    camera_yaw_deg = servo_yaw_left_positive_deg(
        servo_angle_deg, servo_center_deg, servo_positive_left
    )

    angle_left_positive_deg = wrap_degrees(camera_yaw_deg + camera_bearing_deg)
    mode = "bearing"
    distance_m = None
    forward_m = None
    left_m = None

    if qr_size_m is not None and qr_size_m > 0.0:
        camera_matrix = build_camera_matrix(frame_width, frame_height, horizontal_fov_deg)
        pose_forward_m, pose_left_m, pose_distance_m = estimate_target_pose(
            detection, qr_size_m, camera_matrix
        )
        if (
            pose_forward_m is not None
            and pose_left_m is not None
            and pose_distance_m is not None
        ):
            yaw_rad = math.radians(camera_yaw_deg)
            robot_forward_m = (
                math.cos(yaw_rad) * pose_forward_m - math.sin(yaw_rad) * pose_left_m
            )
            robot_left_m = (
                math.sin(yaw_rad) * pose_forward_m + math.cos(yaw_rad) * pose_left_m
            )
            robot_forward_m += camera_forward_offset_m
            robot_left_m += camera_left_offset_m
            angle_left_positive_deg = math.degrees(
                math.atan2(robot_left_m, robot_forward_m)
            )
            mode = "pose"
            distance_m = math.hypot(robot_forward_m, robot_left_m)
            forward_m = robot_forward_m
            left_m = robot_left_m

    if not output_positive_left:
        angle_left_positive_deg *= -1.0
        camera_bearing_deg *= -1.0
        camera_yaw_deg *= -1.0
        if left_m is not None:
            left_m *= -1.0

    return Measurement(
        angle_deg_left_positive=wrap_degrees(angle_left_positive_deg),
        camera_bearing_deg_left_positive=wrap_degrees(camera_bearing_deg),
        camera_yaw_deg_left_positive=wrap_degrees(camera_yaw_deg),
        mode=mode,
        qr_payload=detection.payload,
        center_x=detection.center_x,
        center_y=detection.center_y,
        distance_m=distance_m,
        forward_m=forward_m,
        left_m=left_m,
    )


def draw_overlay(
    frame: "np.ndarray",
    detection: Optional[Detection],
    measurement: Optional[Measurement],
    servo_angle_deg: float,
    stream_url: str,
    output_positive_left: bool,
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

    positive_label = "left" if output_positive_left else "right"
    lines = [
        f"Stream: {stream_url}",
        f"Servo angle: {servo_angle_deg:6.2f} deg",
        f"Output sign: positive {positive_label}",
    ]

    if measurement is None:
        lines.append("Target: not detected")
    else:
        lines.extend(
            [
                f"Target angle: {measurement.angle_deg_left_positive:+7.2f} deg",
                f"Camera bearing: {measurement.camera_bearing_deg_left_positive:+7.2f} deg",
                f"Camera yaw: {measurement.camera_yaw_deg_left_positive:+7.2f} deg",
                f"Mode: {measurement.mode}",
            ]
        )
        if measurement.distance_m is not None:
            lines.append(f"Distance: {measurement.distance_m:6.3f} m")
        if measurement.qr_payload:
            lines.append(f"QR: {measurement.qr_payload}")

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


def emit_report(
    measurement: Optional[Measurement],
    servo_angle_deg: float,
    output_positive_left: bool,
    json_output: bool,
) -> None:
    positive_label = "left" if output_positive_left else "right"

    if json_output:
        payload = {
            "timestamp": time.time(),
            "detected": measurement is not None,
            "servo_angle_deg": servo_angle_deg,
            "positive_direction": positive_label,
        }
        if measurement is not None:
            payload.update(
                {
                    "target_angle_deg": measurement.angle_deg_left_positive,
                    "camera_bearing_deg": measurement.camera_bearing_deg_left_positive,
                    "camera_yaw_deg": measurement.camera_yaw_deg_left_positive,
                    "mode": measurement.mode,
                    "qr_payload": measurement.qr_payload,
                    "distance_m": measurement.distance_m,
                    "forward_m": measurement.forward_m,
                    "left_m": measurement.left_m,
                    "center_px": [measurement.center_x, measurement.center_y],
                }
            )
        print(json.dumps(payload), flush=True)
        return

    if measurement is None:
        print(
            f"target=none servo_deg={servo_angle_deg:.2f} positive={positive_label}",
            flush=True,
        )
        return

    line = (
        f"target_angle_deg={measurement.angle_deg_left_positive:+.2f} "
        f"servo_deg={servo_angle_deg:.2f} "
        f"bearing_deg={measurement.camera_bearing_deg_left_positive:+.2f} "
        f"yaw_deg={measurement.camera_yaw_deg_left_positive:+.2f} "
        f"mode={measurement.mode}"
    )
    if measurement.qr_payload:
        line += f" qr={measurement.qr_payload}"
    if measurement.distance_m is not None:
        line += f" distance_m={measurement.distance_m:.3f}"
    print(line, flush=True)


def open_stream(stream_url: str) -> "cv2.VideoCapture":
    capture = cv2.VideoCapture(stream_url)
    if not capture.isOpened():
        capture.release()
        raise RuntimeError(f"Unable to open camera stream: {stream_url}")
    return capture


def validate_settings(settings: Settings) -> Optional[str]:
    if not settings.stream_url:
        return "SETTINGS.stream_url must not be empty."
    if settings.horizontal_fov_deg <= 0.0 or settings.horizontal_fov_deg >= 179.0:
        return "SETTINGS.horizontal_fov_deg must be between 0 and 179 degrees."
    if settings.report_hz <= 0.0:
        return "SETTINGS.report_hz must be positive."
    if settings.servo_positive not in {"left", "right"}:
        return 'SETTINGS.servo_positive must be "left" or "right".'
    if settings.output_positive not in {"left", "right"}:
        return 'SETTINGS.output_positive must be "left" or "right".'
    return None


def main() -> int:
    settings = SETTINGS
    validation_error = validate_settings(settings)
    if validation_error is not None:
        print(validation_error, file=sys.stderr)
        return 1

    detector = cv2.QRCodeDetector()
    servo_source = ServoAngleSource(settings.servo_angle_deg, settings.servo_angle_file)
    capture: Optional["cv2.VideoCapture"] = None
    smoothed_angle_deg: Optional[float] = None
    next_report_time = 0.0

    try:
        while True:
            if capture is None or not capture.isOpened():
                try:
                    capture = open_stream(settings.stream_url)
                except RuntimeError as exc:
                    print(str(exc), file=sys.stderr)
                    time.sleep(settings.reconnect_delay_s)
                    continue

            ok, frame = capture.read()
            if not ok or frame is None:
                capture.release()
                capture = None
                time.sleep(settings.reconnect_delay_s)
                continue

            frame_height, frame_width = frame.shape[:2]
            servo_angle_deg = servo_source.read()
            detections = detect_qr_codes(detector, frame)
            target = select_detection(
                detections, settings.target_payload, settings.min_area_px
            )
            measurement: Optional[Measurement] = None

            if target is not None:
                measurement = measure_target(
                    detection=target,
                    frame_width=frame_width,
                    frame_height=frame_height,
                    horizontal_fov_deg=settings.horizontal_fov_deg,
                    servo_angle_deg=servo_angle_deg,
                    servo_center_deg=settings.servo_center_deg,
                    servo_positive_left=settings.servo_positive == "left",
                    output_positive_left=settings.output_positive == "left",
                    qr_size_m=settings.qr_size_m,
                    camera_forward_offset_m=settings.camera_forward_offset_m,
                    camera_left_offset_m=settings.camera_left_offset_m,
                )
                measurement.angle_deg_left_positive = smooth_angle_deg(
                    smoothed_angle_deg,
                    measurement.angle_deg_left_positive,
                    settings.smoothing,
                )
                smoothed_angle_deg = measurement.angle_deg_left_positive

            current_time = time.time()
            if current_time >= next_report_time:
                emit_report(
                    measurement=measurement,
                    servo_angle_deg=servo_angle_deg,
                    output_positive_left=settings.output_positive == "left",
                    json_output=settings.json_output,
                )
                next_report_time = current_time + (1.0 / settings.report_hz)

            if not settings.show_preview:
                continue

            preview = draw_overlay(
                frame=frame,
                detection=target,
                measurement=measurement,
                servo_angle_deg=servo_angle_deg,
                stream_url=settings.stream_url,
                output_positive_left=settings.output_positive == "left",
            )
            cv2.imshow("Visual Navigation", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("["):
                servo_source.nudge(-settings.servo_step_deg)
            if key == ord("]"):
                servo_source.nudge(settings.servo_step_deg)
    finally:
        if capture is not None:
            capture.release()
        if settings.show_preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
