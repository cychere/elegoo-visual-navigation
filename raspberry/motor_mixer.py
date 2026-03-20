from __future__ import annotations

from typing import Optional
from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


@dataclass(slots=True)
class DriveCommand:
    angle_deg: float
    speed: float


@dataclass(slots=True)
class WheelCommand:
    left_pwm: int
    right_pwm: int


@dataclass(slots=True)
class MixerSettings:
    max_pwm: int = 255
    minimum_pwm: int = 70
    max_speed: float = 1.0
    max_turn_angle_deg: float = 75.0
    turn_gain: float = 1.0
    pivot_turn_gain: float = 0.65
    pivot_speed_threshold: float = 0.05


def _apply_minimum_pwm(value: float, minimum_pwm: int, max_pwm: int) -> int:
    pwm = int(round(clamp(value, -1.0, 1.0) * max_pwm))
    if pwm == 0:
        return 0
    if abs(pwm) < minimum_pwm:
        return minimum_pwm if pwm > 0 else -minimum_pwm
    return pwm


def mix_drive_command(
    command: DriveCommand,
    settings: Optional[MixerSettings] = None,
) -> WheelCommand:
    settings = settings or MixerSettings()
    speed = clamp(command.speed / max(settings.max_speed, 1e-6), -1.0, 1.0)
    turn = clamp(command.angle_deg / settings.max_turn_angle_deg, -1.0, 1.0)

    if abs(speed) < settings.pivot_speed_threshold:
        left = -turn * settings.pivot_turn_gain
        right = turn * settings.pivot_turn_gain
    else:
        left = speed - (turn * settings.turn_gain)
        right = speed + (turn * settings.turn_gain)

    scale = max(1.0, abs(left), abs(right))
    left /= scale
    right /= scale

    return WheelCommand(
        left_pwm=_apply_minimum_pwm(left, settings.minimum_pwm, settings.max_pwm),
        right_pwm=_apply_minimum_pwm(right, settings.minimum_pwm, settings.max_pwm),
    )
