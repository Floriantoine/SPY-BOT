"""Cinématique inverse et directe d'une patte à 3 articulations.

Repère de la patte : origine sur l'axe de la coxa, x vers l'extérieur de la
patte, y dans le sens trigonométrique, z vers le haut.

Conventions d'angles (radians) :
- coxa  : rotation autour de z, 0 = patte dans l'axe x.
- femur : élévation du fémur par rapport à l'horizontale, positif vers le haut.
- tibia : angle du tibia par rapport au prolongement du fémur, positif vers le
  haut (donc négatif en usage normal : -pi/2 = tibia perpendiculaire au fémur).

La solution retenue est « genou en haut », la configuration habituelle d'un
hexapode.
"""

from dataclasses import dataclass
import math


class UnreachableError(ValueError):
    """La position demandée est hors de portée de la patte."""


@dataclass(frozen=True)
class LegAngles:
    coxa: float
    femur: float
    tibia: float


def leg_ik(x, y, z, coxa_length, femur_length, tibia_length):
    """Retourne les angles articulaires qui placent le pied en (x, y, z)."""
    coxa = math.atan2(y, x)
    r = math.hypot(x, y) - coxa_length  # distance horizontale depuis le fémur
    d = math.hypot(r, z)  # distance fémur -> pied
    if d > femur_length + tibia_length or d < abs(femur_length - tibia_length) or d == 0:
        raise UnreachableError(f"pied hors de portée : ({x:.1f}, {y:.1f}, {z:.1f}), d={d:.1f}")

    # Loi des cosinus dans le plan (r, z).
    cos_knee = (femur_length**2 + tibia_length**2 - d**2) / (2 * femur_length * tibia_length)
    cos_shoulder = (femur_length**2 + d**2 - tibia_length**2) / (2 * femur_length * d)
    knee = math.acos(_clamp(cos_knee))  # angle intérieur fémur/tibia
    femur = math.atan2(z, r) + math.acos(_clamp(cos_shoulder))
    tibia = knee - math.pi
    return LegAngles(coxa, femur, tibia)


def leg_fk(angles, coxa_length, femur_length, tibia_length):
    """Position (x, y, z) du pied pour des angles donnés."""
    r = (
        coxa_length
        + femur_length * math.cos(angles.femur)
        + tibia_length * math.cos(angles.femur + angles.tibia)
    )
    z = femur_length * math.sin(angles.femur) + tibia_length * math.sin(angles.femur + angles.tibia)
    return (r * math.cos(angles.coxa), r * math.sin(angles.coxa), z)


def leg_joint_positions(angles, coxa_length, femur_length, tibia_length):
    """Points coxa, fémur, genou et pied dans le repère patte (pour l'affichage)."""
    c, s = math.cos(angles.coxa), math.sin(angles.coxa)
    knee_r = coxa_length + femur_length * math.cos(angles.femur)
    knee_z = femur_length * math.sin(angles.femur)
    foot = leg_fk(angles, coxa_length, femur_length, tibia_length)
    return [
        (0.0, 0.0, 0.0),
        (coxa_length * c, coxa_length * s, 0.0),
        (knee_r * c, knee_r * s, knee_z),
        foot,
    ]


def _clamp(v):
    return max(-1.0, min(1.0, v))
