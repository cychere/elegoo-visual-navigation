from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class Settings:
    serial_port: str = "/dev/ttyUSB0"
    baud_rate: int = 115200
    stream_url: str = "http://192.168.50.48/stream"
    stream_timeout_s: float = 5.0

    target_marker_ids: tuple[int, ...] = (6,)
    aruco_dictionary_name: str = "DICT_4X4_50"
    marker_size_m: float | None = 0.05

    camera_calibration_path: str = "camera_calibration.npz"
    camera_forward_offset_m: float = 0.0
    camera_left_offset_m: float = 0.0

    min_area_px: float = 50.0
    show_preview: bool = True

    target_distance_m: float = 0.20
    target_distance_tolerance_m: float = 0.03
    heading_kp: float = 0.3
    heading_ki: float = 0.0
    heading_kd: float = 0.01
    distance_kp: float = 0.3
    distance_ki: float = 0.0
    distance_kd: float = 0.01

    target_search_delay_s: float = 1.5
    search_servo_step_deg: int = 26
    search_servo_dwell_s: float = 1.0
    servo_center_angle_deg: float = 72.0
    search_turn_tolerance_deg: float = 5.0
