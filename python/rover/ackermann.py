"""(V, ω) → four steering angles, six motor RPM, six direction bits.

Port of ``AckermannCalculator.calculateFromVelocity`` from the MMS codebase
(`matlab/hardware/AckermannCalculator.m`). Changes must be made in both.

This is pure arithmetic with no I/O, which is deliberate: it can be tested
exhaustively against a golden table without a rover attached, and the driver
that talks to the Teensy never has to think about geometry.

**Three sign conventions here will silently produce a rover that drives wrong,
and none of them are guessable.** They are transcribed, not derived:

1. The right-hand motors are **mounted reversed**, so "forward" is
   ``left=0, right=1`` — not ``0, 0``.
2. The **front** steering servos are mounted opposite the rears, so the two
   front angles are negated on the way out.
3. In a pivot both sides get the *same* direction bit, which by (1) means the
   two sides physically counter-rotate — that is what spins the rover on the
   spot.

There are also two paths into pivot mode: a commanded ``|V| < 1 mm/s`` with a
non-zero ω, and a turn so tight that the inner wheels would have to run
backwards. The second matters — clamping those wheels to zero instead would
stall the inside of the rover mid-corner.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Rig geometry (AckermannCalculator.m lines 103-110). These are physical facts
# about the vehicle, not tuning.
WHEELBASE = 0.6            # m
TRACK_WIDTH = 0.70         # m
MAX_STEERING = 60.0        # deg, mechanical limit
WHEEL_RADIUS = 0.07        # m
GEAR_RATIO = 241           # motor:wheel
MAX_MOTOR_RPM = 3000

PIVOT_THRESHOLD = 1e-3     # m/s below which a commanded V counts as zero
OMEGA_EPS = 1e-6           # rad/s below which ω counts as zero


@dataclass
class DriveCommand:
    """What to send the Teensy for one control tick."""
    wheel_angles: list        # [leftRear, leftFront, rightFront, rightRear] deg
    motor_speeds: list        # [L1 L2 L3 R1 R2 R3] RPM, UNSIGNED
    dir_left: int             # 0/1 — sign lives here, not in the speeds
    dir_right: int
    mode: str = 'ackermann'   # 'straight' | 'ackermann' | 'pivot'

    @property
    def dirs(self) -> list:
        """The six direction bits in motor order, as the firmware expects."""
        return [self.dir_left] * 3 + [self.dir_right] * 3

    def to_dict(self) -> dict:
        return {'wheel_angles': [round(a, 3) for a in self.wheel_angles],
                'motor_speeds': list(self.motor_speeds),
                'dirs': self.dirs, 'mode': self.mode}


def velocity_to_motor_rpm(velocity: float) -> float:
    """m/s at the wheel → motor RPM, through the 241:1 gearbox."""
    wheel_omega = velocity / WHEEL_RADIUS          # rad/s
    wheel_rpm = wheel_omega * (60.0 / (2.0 * math.pi))
    return wheel_rpm * GEAR_RATIO


def _pivot(omega: float) -> DriveCommand:
    """Four-wheel steer about the rover centre; both sides same direction bit."""
    ideal = math.degrees(math.atan(WHEELBASE / TRACK_WIDTH))     # 40.6 deg
    a = min(ideal, MAX_STEERING)
    # Fixed pattern for both senses; the direction bits choose which way it spins.
    angles = [-a, a, -a, a]

    d = 1 if omega > 0 else 0        # CCW reverses both sides; CW drives both
    radius = math.hypot(WHEELBASE / 2.0, TRACK_WIDTH / 2.0)
    speed = velocity_to_motor_rpm(abs(omega) * radius)
    speed = int(round(min(speed, MAX_MOTOR_RPM)))
    return DriveCommand(wheel_angles=angles, motor_speeds=[speed] * 6,
                        dir_left=d, dir_right=d, mode='pivot')


def calculate_from_velocity(v: float, omega: float) -> DriveCommand:
    """Body velocity to wheel commands. `v` m/s forward, `omega` rad/s CCW."""
    # Reversed right-hand motor mounting: forward is (0, 1), not (0, 0).
    dir_left, dir_right = (0, 1) if v >= 0 else (1, 0)
    v_abs = abs(v)

    if abs(v) < PIVOT_THRESHOLD and abs(omega) > OMEGA_EPS:
        return _pivot(omega)

    # ── steering angle from the commanded turn radius ─────────────────────────
    if abs(omega) < OMEGA_EPS:
        steer = 0.0
    else:
        turn_radius = v_abs / abs(omega)
        min_radius = WHEELBASE / math.tan(math.radians(MAX_STEERING))
        turn_radius = max(turn_radius, min_radius)
        steer = math.degrees(math.atan(WHEELBASE / turn_radius))
        if omega < 0:
            steer = -steer
        steer = max(-MAX_STEERING, min(MAX_STEERING, steer))

    if abs(steer) < 0.1:
        speed = min(velocity_to_motor_rpm(v_abs), MAX_MOTOR_RPM)
        speed = max(0, int(round(speed)))
        return DriveCommand(wheel_angles=[0.0, 0.0, 0.0, 0.0],
                            motor_speeds=[speed] * 6,
                            dir_left=dir_left, dir_right=dir_right,
                            mode='straight')

    # ── Ackermann geometry: inner wheel turns more than outer ────────────────
    turn_radius = WHEELBASE / math.tan(abs(math.radians(steer)))
    inner_radius = turn_radius - TRACK_WIDTH / 2.0
    outer_radius = turn_radius + TRACK_WIDTH / 2.0

    if inner_radius <= 0:
        # The turn centre lies inside the track, so the inner wheels would have
        # to run backwards. Clamping them to zero would stall the inside of the
        # rover mid-corner; pivoting is the honest answer.
        return _pivot(omega)

    inner_angle = min(math.degrees(math.atan(WHEELBASE / inner_radius)), MAX_STEERING)
    outer_angle = min(math.degrees(math.atan(WHEELBASE / outer_radius)), MAX_STEERING)

    if steer > 0:                       # left turn: left wheels inner
        left_front = left_rear = inner_angle
        right_front = right_rear = outer_angle
        v_left = v_abs * (inner_radius / turn_radius)
        v_right = v_abs * (outer_radius / turn_radius)
    else:                               # right turn: right wheels inner
        right_front = right_rear = -inner_angle
        left_front = left_rear = -outer_angle
        v_left = v_abs * (outer_radius / turn_radius)
        v_right = v_abs * (inner_radius / turn_radius)

    # The FRONT servos are mounted opposite the rears, so their angles invert.
    angles = [left_rear, -left_front, -right_front, right_rear]
    angles = [max(-MAX_STEERING, min(MAX_STEERING, a)) for a in angles]

    rpm_l = max(0, int(round(min(velocity_to_motor_rpm(v_left), MAX_MOTOR_RPM))))
    rpm_r = max(0, int(round(min(velocity_to_motor_rpm(v_right), MAX_MOTOR_RPM))))

    return DriveCommand(wheel_angles=angles,
                        motor_speeds=[rpm_l] * 3 + [rpm_r] * 3,
                        dir_left=dir_left, dir_right=dir_right,
                        mode='ackermann')


def stop_command() -> DriveCommand:
    """Zero speed, wheels left where they are. Direction bits are irrelevant."""
    return DriveCommand(wheel_angles=[0.0, 0.0, 0.0, 0.0],
                        motor_speeds=[0] * 6, dir_left=0, dir_right=1,
                        mode='straight')
