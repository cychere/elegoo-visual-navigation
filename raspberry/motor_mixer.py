import math
from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


@dataclass(slots=True)
class RobotCommand:
    turn_effort: float
    speed_effort: float


@dataclass(slots=True)
class WheelCommand:
    left_pwm: int
    right_pwm: int


@dataclass(slots=True)
class MixerSettings:
    max_pwm: int = 100
    min_pwm: int = 40
    max_speed_effort: float = 1.0
    max_turn_effort: float = math.radians(75.0)
    turn_gain: float = 1.0


def _apply_pwm(value: float, min_pwm: int, max_pwm: int) -> int:
    pwm = int(round(value * max_pwm))
    if pwm == 0:
        return 0
    if abs(pwm) < min_pwm:
        return min_pwm if pwm > 0 else -min_pwm
    return pwm


def mix_drive_command(command: RobotCommand) -> WheelCommand:
    settings = MixerSettings()
    speed_effort = clamp(command.speed_effort / settings.max_speed_effort, -1.0, 1.0)
    turn_effort = clamp(command.turn_effort / settings.max_turn_effort, -1.0, 1.0)

    left = speed_effort - (turn_effort * settings.turn_gain)
    right = speed_effort + (turn_effort * settings.turn_gain)

    scale = max(1.0, abs(left), abs(right))
    left /= scale
    right /= scale

    return WheelCommand(
        left_pwm=_apply_pwm(left, settings.min_pwm, settings.max_pwm),
        right_pwm=_apply_pwm(right, settings.min_pwm, settings.max_pwm),
    )
