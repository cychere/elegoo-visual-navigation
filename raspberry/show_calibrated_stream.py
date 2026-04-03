from __future__ import annotations

import argparse
import sys
import time
import socket
from pathlib import Path
from urllib.error import URLError

import cv2

from vision import load_camera_calibration, open_stream


DEFAULT_STREAM_URL = "http://192.168.50.48/stream"
DEFAULT_TIMEOUT_S = 5.0
DEFAULT_RECONNECT_DELAY_S = 1.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display the live camera stream with calibration distortion correction applied."
    )
    parser.add_argument(
        "--stream-url",
        default=DEFAULT_STREAM_URL,
        help=f"MJPEG stream URL. Default: {DEFAULT_STREAM_URL}",
    )
    parser.add_argument(
        "--calibration",
        default=str(Path(__file__).with_name("camera_calibration.npz")),
        help="Path to the camera calibration .npz file.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"Stream open/read timeout in seconds. Default: {DEFAULT_TIMEOUT_S}",
    )
    parser.add_argument(
        "--reconnect-delay-s",
        type=float,
        default=DEFAULT_RECONNECT_DELAY_S,
        help=f"Delay before reconnecting after a stream failure. Default: {DEFAULT_RECONNECT_DELAY_S}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        calibration = load_camera_calibration(args.calibration)
    except (FileNotFoundError, ValueError, OSError) as exc:
        print(f"Camera calibration error: {exc}", file=sys.stderr)
        return 1

    stream = None
    try:
        while True:
            if stream is None:
                try:
                    stream = open_stream(args.stream_url, args.timeout_s)
                except RuntimeError as exc:
                    print(str(exc), file=sys.stderr)
                    time.sleep(args.reconnect_delay_s)
                    continue

            try:
                frame = stream.read()
            except RuntimeError as exc:
                print(str(exc), file=sys.stderr)
                stream.close()
                stream = None
                time.sleep(args.reconnect_delay_s)
                continue
            except (URLError, TimeoutError, socket.timeout, OSError) as exc:
                print(f"Camera stream read failed: {exc}", file=sys.stderr)
                stream.close()
                stream = None
                time.sleep(args.reconnect_delay_s)
                continue

            frame_height, frame_width = frame.shape[:2]
            camera_matrix = calibration.camera_matrix_for_frame(frame_width, frame_height)
            calibrated = cv2.undistort(
                frame,
                camera_matrix,
                calibration.distortion_coeffs,
            )
            comparison = cv2.hconcat([frame, calibrated])
            cv2.imshow("Original | Calibrated", comparison)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
    finally:
        if stream is not None:
            stream.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
