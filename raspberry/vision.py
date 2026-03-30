from __future__ import annotations

import cv2
import math
import socket
import numpy as np
from typing import Optional
from dataclasses import dataclass
from urllib.parse import urlparse
from urllib.error import URLError


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


class VideoCaptureStream:
    def __init__(self, stream_url: str, timeout_s: float) -> None:
        self._capture: Optional[cv2.VideoCapture] = None
        timeout_ms = int(max(timeout_s, 0.0) * 1000.0)
        open_errors: list[str] = []

        for source, backend_name, api_preference in capture_candidates(stream_url):
            capture = cv2.VideoCapture()

            if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
                capture.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, timeout_ms)
            if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
                capture.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, timeout_ms)
            if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            try:
                opened = capture.open(source, api_preference)
            except cv2.error as exc:
                open_errors.append(f"{backend_name}: {exc}")
                capture.release()
                continue

            if opened:
                self._capture = capture
                return

            open_errors.append(f"{backend_name}: open() returned false")
            capture.release()

        error_details = "; ".join(open_errors) if open_errors else "no backends attempted"
        raise RuntimeError(f"Camera stream failed to open. {error_details}")

    def read(self) -> "np.ndarray":
        if self._capture is None:
            raise RuntimeError("Camera stream is not open.")

        success, frame = self._capture.read()
        if not success or frame is None:
            raise RuntimeError("Camera stream closed.")
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None


def capture_candidates(stream_url: str) -> list[tuple[str, str, int]]:
    candidates: list[tuple[str, str, int]] = []
    parsed = urlparse(stream_url)
    is_http_stream = parsed.scheme in {"http", "https"}

    if is_http_stream and hasattr(cv2, "CAP_FFMPEG"):
        candidates.append((stream_url, "FFmpeg", cv2.CAP_FFMPEG))

    gstreamer_pipeline = build_gstreamer_mjpeg_pipeline(stream_url)
    if gstreamer_pipeline is not None and hasattr(cv2, "CAP_GSTREAMER"):
        candidates.append((gstreamer_pipeline, "GStreamer MJPEG pipeline", cv2.CAP_GSTREAMER))

    candidates.append((stream_url, "default backend", cv2.CAP_ANY))
    return candidates


def build_gstreamer_mjpeg_pipeline(stream_url: str) -> Optional[str]:
    parsed = urlparse(stream_url)
    if parsed.scheme not in {"http", "https"}:
        return None

    # ESP32-CAM style `/stream` endpoints are multipart MJPEG and need explicit demuxing.
    return (
        f"souphttpsrc location={stream_url} is-live=true do-timestamp=true ! "
        "multipartdemux ! jpegdec ! videoconvert ! "
        "appsink drop=true max-buffers=1 sync=false"
    )


def wrap_degrees(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


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


def draw_overlay(
    frame: "np.ndarray",
    detection: Optional[Detection],
    measurement: Optional[VisualMeasurement],
    decision,
    settings,
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
        f"Robot angle: {decision.robot.angle_deg:+6.1f} deg",
        f"Robot speed: {decision.robot.speed:.2f}",
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


def open_stream(stream_url: str, timeout_s: float) -> VideoCaptureStream:
    try:
        return VideoCaptureStream(stream_url, timeout_s)
    except (RuntimeError, URLError, TimeoutError, socket.timeout, OSError) as exc:
        raise RuntimeError(f"Unable to open camera stream: {stream_url} ({exc})") from exc
