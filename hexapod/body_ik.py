"""Cinématique inverse du corps : translation et rotation du châssis.

Les pieds restent fixes au sol pendant que le corps bouge ; on calcule donc,
pour chaque patte, où se trouve le pied vu depuis le corps déplacé, puis dans
le repère de la patte.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class BodyPose:
    """Pose du corps par rapport à sa position neutre (mm et radians)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0  # autour de x
    pitch: float = 0.0  # autour de y
    yaw: float = 0.0  # autour de z


def rotation_matrix(roll, pitch, yaw):
    """R = Rz(yaw) · Ry(pitch) · Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def foot_in_body(foot_world, pose):
    """Pied exprimé dans le repère du corps déplacé : R^T · (p - t)."""
    r = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
    dx = foot_world[0] - pose.x
    dy = foot_world[1] - pose.y
    dz = foot_world[2] - pose.z
    return (
        r[0][0] * dx + r[1][0] * dy + r[2][0] * dz,
        r[0][1] * dx + r[1][1] * dy + r[2][1] * dz,
        r[0][2] * dx + r[1][2] * dy + r[2][2] * dz,
    )


def body_to_leg_frame(foot_body, leg):
    """Passe du repère corps au repère de la patte `leg` (LegConfig)."""
    dx = foot_body[0] - leg.mount_x
    dy = foot_body[1] - leg.mount_y
    c, s = math.cos(leg.mount_angle), math.sin(leg.mount_angle)
    return (c * dx + s * dy, -s * dx + c * dy, foot_body[2])


def leg_to_body_frame(foot_leg, leg):
    c, s = math.cos(leg.mount_angle), math.sin(leg.mount_angle)
    return (
        leg.mount_x + c * foot_leg[0] - s * foot_leg[1],
        leg.mount_y + s * foot_leg[0] + c * foot_leg[1],
        foot_leg[2],
    )
