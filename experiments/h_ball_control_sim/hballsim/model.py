from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PlantParameters:
    """Physical parameters for the one-dimensional rolling-ball model."""

    gravity: float = 9.80665
    rolling_factor: float = 5.0 / 7.0
    beam_center_offset: float = 0.08
    beam_misalignment: float = 0.0
    viscous_damping: float = 0.20
    coulomb_accel: float = 0.0
    coulomb_smoothing_speed: float = 0.005
    static_friction_accel: float = 0.0
    static_friction_speed: float = 0.003
    turn_friction_accel_per_yaw: float = 0.0


def ball_acceleration(
    *,
    position: float,
    velocity: float,
    beam_angle: float,
    body_pitch: float,
    longitudinal_accel: float,
    lateral_accel: float,
    yaw_rate: float,
    params: PlantParameters,
) -> float:
    """Return ball acceleration along the beam in the vehicle frame.

    Positive beam angle is defined to accelerate the ball toward positive
    position. The yaw term is the longitudinal component of centrifugal
    acceleration about the vehicle reference point.
    """

    world_beam_angle = beam_angle + body_pitch
    tangent_accel = (
        params.gravity * math.sin(world_beam_angle)
        - longitudinal_accel * math.cos(world_beam_angle)
        + yaw_rate**2 * (params.beam_center_offset + position)
        - lateral_accel * math.sin(params.beam_misalignment)
    )
    drive_accel = params.rolling_factor * tangent_accel
    if (
        abs(velocity) < params.static_friction_speed
        and abs(drive_accel) <= params.static_friction_accel
    ):
        return 0.0

    if abs(velocity) < params.static_friction_speed and drive_accel != 0.0:
        motion_direction = math.copysign(1.0, drive_accel)
    else:
        motion_direction = math.tanh(
            velocity / params.coulomb_smoothing_speed
        )
    friction = params.viscous_damping * velocity
    friction += (
        params.coulomb_accel
        + params.turn_friction_accel_per_yaw * abs(yaw_rate)
    ) * motion_direction
    return drive_accel - friction
