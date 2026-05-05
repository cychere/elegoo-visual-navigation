from __future__ import annotations

import cv2
import math
import numpy as np
from dataclasses import dataclass
from pathlib import Path
from urllib.request import Request, urlopen

from angles import wrap_degrees


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
    angle_rad: float
    marker_id: int
    center_x: float
    center_y: float
    distance_m: float | None


@dataclass(slots=True)
class CameraCalibration:
    camera_matrix: "np.ndarray"
    distortion_coeffs: "np.ndarray"
    image_width: int
    image_height: int
    reprojection_error: float

    def camera_matrix_for_frame(self, frame_width: int, frame_height: int) -> "np.ndarray":
        scaled = self.camera_matrix.astype(np.float32).copy()
        scale_x = frame_width / float(self.image_width)
        scale_y = frame_height / float(self.image_height)
        scaled[0, 0] *= scale_x
        scaled[0, 2] *= scale_x
        scaled[1, 1] *= scale_y
        scaled[1, 2] *= scale_y
        return scaled


def load_camera_calibration(calibration_path: str | Path) -> CameraCalibration:
    with np.load(Path(calibration_path).expanduser(), allow_pickle=False) as data:
        return CameraCalibration(
            camera_matrix=np.asarray(data["camera_matrix"], dtype=np.float32).reshape(3, 3),
            distortion_coeffs=np.asarray(data["distortion_coeffs"], dtype=np.float32).reshape(
                -1, 1
            ),
            image_width=int(np.asarray(data["image_width"]).item()),
            image_height=int(np.asarray(data["image_height"]).item()),
            reprojection_error=float(np.asarray(data["reprojection_error"]).item()),
        )


class MjpegStream:
    def __init__(self, stream_url: str, timeout_s: float) -> None:
        request = Request(
            stream_url,
            headers={"User-Agent": "elegoo-visual-navigation"},
        )
        self._response = urlopen(request, timeout=timeout_s)
        self._buffer = bytearray()
        self._chunk_size = 8192

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

            self._buffer.extend(self._response.read(self._chunk_size))

    def close(self) -> None:
        self._response.close()


def horizontal_bearing_deg(
    center_x: float,
    center_y: float,
    frame_width: int,
    frame_height: int,
    camera_calibration: CameraCalibration,
) -> float:
    camera_matrix = camera_calibration.camera_matrix_for_frame(frame_width, frame_height)
    undistorted = cv2.undistortPoints(
        np.array([[[center_x, center_y]]], dtype=np.float32),
        camera_matrix,
        camera_calibration.distortion_coeffs,
    )
    normalized_x = float(undistorted[0, 0, 0])
    return math.degrees(math.atan2(-normalized_x, 1.0))


def build_aruco_detector(dictionary_name: str) -> object:
    aruco = cv2.aruco
    parameters = aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 35
    parameters.adaptiveThreshWinSizeStep = 4
    parameters.minMarkerPerimeterRate = 0.02
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    parameters.errorCorrectionRate = 0.7

    dictionary = aruco.getPredefinedDictionary(getattr(aruco, dictionary_name))
    return aruco.ArucoDetector(dictionary, parameters)


def detect_aruco_markers(detector: object, frame: "np.ndarray") -> list[Detection]:
    corners_list, ids, _rejected = detector.detectMarkers(frame)

    if ids is None:
        return []

    detections: list[Detection] = []
    for marker_id, corners in zip(np.asarray(ids, dtype=np.int32).reshape(-1), corners_list):
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


def estimate_marker_pose(
    detection: Detection,
    marker_size_m: float,
    camera_matrix: "np.ndarray",
    distortion_coeffs: "np.ndarray",
) -> tuple[float, float] | None:
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

    success, _rvec, tvec = cv2.solvePnP(
        object_points,
        detection.corners.astype(np.float32),
        camera_matrix,
        distortion_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        return None

    tx = float(tvec[0, 0])
    tz = float(tvec[2, 0])

    if tz <= 0.0:
        return None

    return tz, -tx


def measure_target(
    detection: Detection,
    frame_width: int,
    frame_height: int,
    camera_calibration: CameraCalibration,
    marker_size_m: float | None,
    camera_forward_offset_m: float,
    camera_left_offset_m: float,
) -> VisualMeasurement:
    angle_deg = horizontal_bearing_deg(
        detection.center_x,
        detection.center_y,
        frame_width,
        frame_height,
        camera_calibration,
    )
    distance_m = None

    if marker_size_m is not None and marker_size_m > 0.0:
        camera_matrix = camera_calibration.camera_matrix_for_frame(frame_width, frame_height)
        pose = estimate_marker_pose(
            detection,
            marker_size_m,
            camera_matrix,
            camera_calibration.distortion_coeffs,
        )
        if pose is not None:
            pose_forward_m, pose_left_m = pose
            robot_forward_m = pose_forward_m + camera_forward_offset_m
            robot_left_m = pose_left_m + camera_left_offset_m
            angle_deg = math.degrees(math.atan2(robot_left_m, robot_forward_m))
            distance_m = math.hypot(robot_forward_m, robot_left_m)

    angle_deg = wrap_degrees(angle_deg)

    return VisualMeasurement(
        angle_deg=angle_deg,
        angle_rad=math.radians(angle_deg),
        marker_id=detection.marker_id,
        center_x=detection.center_x,
        center_y=detection.center_y,
        distance_m=distance_m,
    )


def draw_overlay(
    frame: "np.ndarray",
    detection: Detection | None,
    measurement: VisualMeasurement | None,
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
        f"Mode: {decision.mode}",
        f"Target visible: {'yes' if decision.target_visible else 'no'}",
        f"Turn effort: {decision.robot.turn_effort:+.3f}",
        f"Speed effort: {decision.robot.speed_effort:+.2f}",
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


def open_stream(stream_url: str, timeout_s: float) -> MjpegStream:
    return MjpegStream(stream_url, timeout_s)
