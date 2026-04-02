from __future__ import annotations

"""
Calibrate the Raspberry camera from photos of a 9x6 chessboard.

Usage example:
    python3 raspberry/camera_calibrate.py "calibration/*.jpeg" --square-size-mm 23

How to capture the photos:
    - Use a rigid, flat printed checkerboard with 9x6 inner corners.
      That means the paper has 10x7 alternating black/white squares.
    - Measure the edge length of one square, not the whole board.
      Pass that measurement with --square-size-mm or --square-size-m.
    - Capture at least 12-20 sharp photos at the same camera resolution,
      focus, and lens setting that navigation will use.
    - Move the board around the frame: center, edges, and corners.
    - Tilt and rotate it so the set includes different distances and angles.
    - Keep the whole board visible in each accepted image and avoid motion blur.
    - Do not collect only straight-on, centered shots; that produces a weak calibration.

The script writes `camera_calibration.npz`, which `vision.py` loads at runtime.
"""

import argparse
import glob
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calibrate a camera from photos of a 9x6 chessboard."
    )
    parser.add_argument(
        "images",
        nargs="+",
        help="Image paths or glob patterns, for example calibration/*.jpg",
    )
    parser.add_argument(
        "--pattern-cols",
        type=int,
        default=9,
        help="Chessboard inner corners across the board width. Default: 9",
    )
    parser.add_argument(
        "--pattern-rows",
        type=int,
        default=6,
        help="Chessboard inner corners across the board height. Default: 6",
    )
    square_group = parser.add_mutually_exclusive_group(required=True)
    square_group.add_argument(
        "--square-size-mm",
        type=float,
        help="Measured edge length of one square in millimeters.",
    )
    square_group.add_argument(
        "--square-size-m",
        type=float,
        help="Measured edge length of one square in meters.",
    )
    parser.add_argument(
        "--output",
        default=str(Path(__file__).with_name("camera_calibration.npz")),
        help="Output calibration file. Default: raspberry/camera_calibration.npz",
    )
    return parser.parse_args()


def resolve_image_paths(patterns: list[str]) -> list[Path]:
    resolved: list[Path] = []
    for pattern in patterns:
        matches = sorted(Path(path) for path in glob.glob(pattern, recursive=True))
        if matches:
            resolved.extend(matches)
            continue

        candidate = Path(pattern)
        if candidate.is_file():
            resolved.append(candidate)

    unique_paths: list[Path] = []
    seen: set[Path] = set()
    for path in resolved:
        normalized = path.resolve()
        if normalized in seen:
            continue
        seen.add(normalized)
        unique_paths.append(normalized)

    return unique_paths


def square_size_m_from_args(args: argparse.Namespace) -> float:
    if args.square_size_m is not None:
        square_size_m = float(args.square_size_m)
    else:
        square_size_m = float(args.square_size_mm) / 1000.0

    if square_size_m <= 0.0:
        raise SystemExit("Square size must be greater than zero.")

    return square_size_m


def build_object_points(pattern_cols: int, pattern_rows: int, square_size_m: float) -> np.ndarray:
    grid = np.zeros((pattern_rows * pattern_cols, 3), dtype=np.float32)
    grid[:, :2] = (
        np.mgrid[0:pattern_cols, 0:pattern_rows].T.reshape(-1, 2) * square_size_m
    )
    return grid


def detect_corners(gray: np.ndarray, pattern_size: tuple[int, int]) -> np.ndarray | None:
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags=flags)
        if found:
            return corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=flags)
    if not found:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(
        gray,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=criteria,
    )
    return refined.astype(np.float32)


def main() -> int:
    args = parse_args()
    image_paths = resolve_image_paths(args.images)
    if not image_paths:
        raise SystemExit("No images matched the provided paths or glob patterns.")

    pattern_size = (args.pattern_cols, args.pattern_rows)
    square_size_m = square_size_m_from_args(args)
    object_template = build_object_points(args.pattern_cols, args.pattern_rows, square_size_m)

    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    accepted_images: list[str] = []
    image_size: tuple[int, int] | None = None

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            print(f"Skipping unreadable image: {image_path}")
            continue

        height, width = image.shape[:2]
        current_size = (width, height)
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            print(
                f"Skipping {image_path}: image size {current_size} does not match "
                f"the first accepted size {image_size}."
            )
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners = detect_corners(gray, pattern_size)
        if corners is None:
            print(
                f"Skipping {image_path}: {args.pattern_cols}x{args.pattern_rows} "
                "chessboard was not detected."
            )
            continue

        object_points.append(object_template.copy())
        image_points.append(corners)
        accepted_images.append(str(image_path))
        print(f"Accepted {image_path}")

    if len(accepted_images) < 3 or image_size is None:
        raise SystemExit(
            "Calibration needs at least 3 accepted images with detected chessboard corners."
        )

    reprojection_error, camera_matrix, distortion_coeffs, _rvecs, _tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        camera_matrix=camera_matrix.astype(np.float32),
        distortion_coeffs=distortion_coeffs.astype(np.float32),
        image_width=np.int32(image_size[0]),
        image_height=np.int32(image_size[1]),
        pattern_columns=np.int32(args.pattern_cols),
        pattern_rows=np.int32(args.pattern_rows),
        square_size_m=np.float32(square_size_m),
        reprojection_error=np.float32(reprojection_error),
        accepted_images=np.asarray(accepted_images),
    )

    print(f"Saved calibration to {output_path}")
    print(f"Accepted images: {len(accepted_images)}")
    print(f"Image size: {image_size[0]}x{image_size[1]}")
    print(f"Mean reprojection error: {reprojection_error:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
