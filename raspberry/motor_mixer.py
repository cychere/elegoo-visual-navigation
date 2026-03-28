from dataclasses import dataclass


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


@dataclass(slots=True)
class RobotCommand:
    angle_deg: float
    speed: float


@dataclass(slots=True)
class WheelCommand:
    left_pwm: int
    right_pwm: int


@dataclass(slots=True)
class MixerSettings:
    max_pwm: int = 255
    min_pwm: int = 70
    max_speed: float = 1.0
    max_angle_deg: float = 75.0
    turn_gain: float = 1.0
    pivot_turn_gain: float = 0.65
    pivot_speed_threshold: float = 0.05


def _apply_pwm(value: float, min_pwm: int, max_pwm: int) -> int:
    pwm = int(round(value * max_pwm))
    if pwm == 0:
        return 0
    if abs(pwm) < min_pwm:
        return min_pwm if pwm > 0 else -min_pwm
    return pwm


def mix_drive_command(command: RobotCommand) -> WheelCommand:
    settings = MixerSettings()
    speed = clamp(command.speed / settings.max_speed, -1.0, 1.0)
    turn = clamp(command.angle_deg / settings.max_angle_deg, -1.0, 1.0)

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
        left_pwm=_apply_pwm(left, settings.min_pwm, settings.max_pwm),
        right_pwm=_apply_pwm(right, settings.min_pwm, settings.max_pwm)
    )
