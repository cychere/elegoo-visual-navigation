from __future__ import annotations

import time
import serial
from typing import Optional
from dataclasses import dataclass


def clamp(value: int, minimum: int, maximum: int) -> int:
    return max(minimum, min(value, maximum))


@dataclass(slots=True)
class SensorReading:
    yaw_deg: float
    distance_cm: int
    timestamp_s: float


class ArduinoLink:
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baud_rate: int = 115200,
        timeout_s: float = 0.05,
        startup_delay_s: float = 2.0,
    ) -> None:
        if serial is None:
            raise ModuleNotFoundError(
                "pyserial is required to talk to the Arduino. Install it with `pip install pyserial`."
            )
        self._serial = serial.Serial(port, baud_rate, timeout=timeout_s)
        self._latest_reading: Optional[SensorReading] = None
        time.sleep(startup_delay_s)
        self.flush_input()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "ArduinoLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def flush_input(self) -> None:
        self._serial.reset_input_buffer()

    def _parse_line(self, line: str) -> Optional[SensorReading]:
        if not line.startswith("SENSOR "):
            return None

        parts = line.split()
        if len(parts) != 3:
            return None

        try:
            yaw_deg = float(parts[1])
            distance_cm = int(parts[2])
        except ValueError:
            return None

        return SensorReading(
            yaw_deg=yaw_deg,
            distance_cm=distance_cm,
            timestamp_s=time.time(),
        )

    def read_latest(self, max_lines: int = 64) -> Optional[SensorReading]:
        latest = self._latest_reading
        lines_read = 0

        while lines_read < max_lines and self._serial.in_waiting > 0:
            raw = self._serial.readline()
            lines_read += 1
            if not raw:
                continue

            reading = self._parse_line(raw.decode("ascii", errors="ignore").strip())
            if reading is not None:
                latest = reading

        self._latest_reading = latest
        return latest

    def wait_for_reading(self, timeout_s: float = 2.0) -> SensorReading:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            reading = self.read_latest()
            if reading is not None:
                return reading

            raw = self._serial.readline()
            if not raw:
                continue

            parsed = self._parse_line(raw.decode("ascii", errors="ignore").strip())
            if parsed is not None:
                self._latest_reading = parsed
                return parsed

        raise TimeoutError("Timed out waiting for SENSOR data from the Arduino.")

    def send_motor(self, left_pwm: int, right_pwm: int) -> None:
        left_pwm = clamp(left_pwm, -255, 255)
        right_pwm = clamp(right_pwm, -255, 255)
        self._serial.write(f"MOTOR {left_pwm} {right_pwm}\n".encode("ascii"))
        self._serial.flush()

    def send_servo(self, angle_deg: int) -> None:
        angle_deg = clamp(angle_deg, 0, 180)
        self._serial.write(f"SERVO {angle_deg}\n".encode("ascii"))
        self._serial.flush()

    def stop(self) -> None:
        self.send_motor(0, 0)
