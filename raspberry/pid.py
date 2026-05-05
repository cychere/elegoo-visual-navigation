from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class PIDController:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    previous_error: float | None = None
    previous_time_s: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None
        self.previous_time_s = None

    def update(self, error: float, now_s: float) -> float:
        derivative = 0.0

        if self.previous_time_s is not None and self.previous_error is not None:
            dt_s = now_s - self.previous_time_s
            self.integral += error * dt_s
            derivative = (error - self.previous_error) / dt_s

        self.previous_error = error
        self.previous_time_s = now_s
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
