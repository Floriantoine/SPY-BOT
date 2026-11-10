"""Cinématique inverse du corps : translation et rotation du châssis.

Les pieds restent fixes au sol pendant que le corps bouge ; on calcule donc,
pour chaque patte, où se trouve le pied vu depuis le corps déplacé, puis dans
le repère de la patte.
"""

import math


class BodyPose:
    """Pose du corps par rapport à sa position neutre (mm et radians).

    roll autour de x, pitch autour de y, yaw autour de z.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


_last_rotation = (None, None)


def rotation_matrix(roll, pitch, yaw):
    """R = Rz(yaw) · Ry(pitch) · Rx(roll). La dernière est gardée : le plus
    souvent, la pose du corps ne change pas d'un calcul à l'autre."""
    global _last_rotation
    key = (roll, pitch, yaw)
    if _last_rotation[0] == key:
        return _last_rotation[1]
    r = _rotation(roll, pitch, yaw)
    _last_rotation = (key, r)
    return r


def _rotation(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def foot_in_body(foot_world, pose, r=None):
    """Pied exprimé dans le repère du corps déplacé : R^T · (p - t).
    `r` : rotation_matrix de la pose, si elle est déjà calculée."""
    if r is None:
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
    c, s = _trig(leg)
    return (c * dx + s * dy, -s * dx + c * dy, foot_body[2])


def _trig(leg):
    """cos et sin de l'orientation de la patte, gardés (calculés une fois)."""
    t = getattr(leg, "_trig", None)
    if t is None or t[0] != leg.mount_angle:
        a = leg.mount_angle
        t = leg._trig = (a, math.cos(a), math.sin(a))
    return t[1], t[2]


def leg_to_body_frame(foot_leg, leg):
    c, s = math.cos(leg.mount_angle), math.sin(leg.mount_angle)
    return (
        leg.mount_x + c * foot_leg[0] - s * foot_leg[1],
        leg.mount_y + s * foot_leg[0] + c * foot_leg[1],
        foot_leg[2],
    )
