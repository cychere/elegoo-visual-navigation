import time
import serial
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
        self._serial = serial.Serial(port, baud_rate, timeout=timeout_s)
        self._latest_reading = SensorReading(0, 0, 0)
        time.sleep(startup_delay_s)
        self._serial.reset_input_buffer()

    def close(self) -> None:
        self._serial.close()

    def __enter__(self) -> "ArduinoLink":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _parse_line(self, line: str) -> SensorReading:

        parts = line.split()
        yaw = float(parts[1])
        distance = int(parts[2])

        return SensorReading(
            yaw_deg=yaw,
            distance_cm=distance,
            timestamp_s=time.time(),
        )

    def read_latest(self, max_lines: int = 60) -> SensorReading:
        latest = self._latest_reading
        lines_read = 0

        while lines_read < max_lines and self._serial.in_waiting > 0:
            raw = self._serial.readline()
            lines_read += 1
            latest = self._parse_line(raw.decode("ascii", errors="ignore").strip())

        self._latest_reading = latest
        return latest

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
