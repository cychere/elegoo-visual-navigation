#!/usr/bin/env python3
"""Calculate average angular speed values from a sensor log."""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ANGULAR_SPEED_RE = re.compile(r"([+-]?\d+(?:\.\d+)?)\s+deg/s\b")


def read_angular_speeds(path: Path) -> list[float]:
    speeds: list[float] = []

    with path.open("r", encoding="utf-8") as file:
        for line_number, line in enumerate(file, start=1):
            match = ANGULAR_SPEED_RE.search(line)
            if match is None:
                continue

            try:
                speeds.append(float(match.group(1)))
            except ValueError as exc:
                raise ValueError(f"Invalid speed on line {line_number}: {line.rstrip()}") from exc

    return speeds


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Average angular speed readings from lines like '-32.435070 deg/s'."
    )
    parser.add_argument(
        "file",
        nargs="?",
        default=Path("/Volumes/cyc/Desktop/share/new.txt"),
        type=Path,
        help="Path to the sensor log file. Defaults to new.txt beside this script.",
    )
    args = parser.parse_args()

    speeds = read_angular_speeds(args.file)
    if not speeds:
        raise SystemExit(f"No angular speed values found in {args.file}")

    signed_average = sum(speeds) / len(speeds)
    magnitude_average = sum(abs(speed) for speed in speeds) / len(speeds)

    print(f"Readings: {len(speeds)}")
    print(f"Average angular velocity: {signed_average:.6f} deg/s")
    print(f"Average angular speed magnitude: {magnitude_average:.6f} deg/s")


if __name__ == "__main__":
    main()
