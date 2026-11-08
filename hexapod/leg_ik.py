"""Cinématique inverse et directe d'une patte à 3 articulations.

Repère de la patte : origine sur l'axe de la coxa, x vers l'extérieur de la
patte, y dans le sens trigonométrique, z vers le haut.

Conventions d'angles (radians) :
- coxa  : rotation autour de z, 0 = patte dans l'axe x.
- femur : élévation du fémur par rapport à l'horizontale, positif vers le haut.
- tibia : angle du tibia par rapport au prolongement du fémur, positif vers le
  haut (donc négatif en usage normal : -pi/2 = tibia perpendiculaire au fémur).

La solution retenue est « genou en haut », la configuration habituelle d'un
hexapode. Compatible MicroPython (ESP32).
"""

import math

try:
    import micropython  # @micropython.native : compilé en code machine sur l'ESP32
except ImportError:  # CPython (PC, simulation) : décorateur sans effet
    class micropython:
        @staticmethod
        def native(f):
            return f


class UnreachableError(ValueError):
    """La position demandée est hors de portée de la patte."""


class LegAngles:
    def __init__(self, coxa, femur, tibia):
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

    def __repr__(self):
        return "LegAngles(%.4f, %.4f, %.4f)" % (self.coxa, self.femur, self.tibia)


@micropython.native
def leg_ik(x, y, z, coxa_length, femur_length, tibia_length):
    """Retourne les angles articulaires qui placent le pied en (x, y, z)."""
    # Écrit avec peu d'opérations : sur l'ESP32 chaque calcul flottant crée
    # un objet (et le ramasse-miettes coûte cher) ; appelée 6 fois par tick.
    k = _consts(coxa_length, femur_length, tibia_length)
    r = _sqrt(x * x + y * y) - coxa_length  # distance horizontale depuis le fémur
    dd = r * r + z * z
    d = _sqrt(dd)  # distance fémur -> pied
    if d > k[3] or d < k[4] or d == 0:
        raise UnreachableError("pied hors de portée : (%.1f, %.1f, %.1f), d=%.1f" % (x, y, z, d))

    # Loi des cosinus dans le plan (r, z).
    cos_knee = (k[5] - dd) / k[6]
    cos_shoulder = (k[7] + dd) / (k[8] * d)
    cos_knee = -1.0 if cos_knee < -1.0 else 1.0 if cos_knee > 1.0 else cos_knee
    cos_shoulder = -1.0 if cos_shoulder < -1.0 else 1.0 if cos_shoulder > 1.0 else cos_shoulder
    # acos(cos_knee) : angle intérieur fémur/tibia.
    return LegAngles(_atan2(y, x), _atan2(z, r) + _acos(cos_shoulder), _acos(cos_knee) - _PI)


_k = (None, None, None)


def _consts(c, f, t):
    """Termes de leg_ik qui ne dépendent que des longueurs (gardés)."""
    global _k
    k = _k
    if k[0] != c or k[1] != f or k[2] != t:
        k = _k = (c, f, t, f + t, abs(f - t), f * f + t * t, 2 * f * t, f * f - t * t, 2 * f)
    return k


_sqrt, _atan2, _acos, _PI = math.sqrt, math.atan2, math.acos, math.pi


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
