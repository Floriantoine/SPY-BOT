"""Géométrie du robot et câblage des servos.

Repère du corps : x vers l'avant, y vers la gauche, z vers le haut, origine au
centre du châssis. Toutes les longueurs sont en millimètres, les angles en
radians sauf mention contraire.
"""

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class JointServo:
    """Lien entre un angle articulaire (IK) et un canal de servo PCA9685.

    angle_servo_deg = 90 + direction * (degrés(angle_ik) - zero_deg)
    """

    board: int  # index de la carte PCA9685 (0 ou 1, 16 canaux chacune)
    channel: int
    zero_deg: float = 0.0  # angle IK (en degrés) qui place le servo à 90°
    direction: int = 1  # +1 ou -1 selon le sens de montage
    trim_deg: float = 0.0  # réglage fin mesuré sur le robot réel
    min_deg: float = 0.0
    max_deg: float = 180.0


@dataclass(frozen=True)
class LegConfig:
    name: str
    mount_x: float  # position de l'axe coxa dans le repère corps
    mount_y: float
    mount_angle: float  # orientation de la patte (rad), 0 = vers l'avant
    coxa: JointServo
    femur: JointServo
    tibia: JointServo


@dataclass(frozen=True)
class HexapodConfig:
    coxa_length: float = 50.0
    femur_length: float = 80.0
    tibia_length: float = 120.0
    # Position neutre des pieds : distance horizontale depuis l'axe coxa et
    # hauteur sous le corps.
    stance_radius: float = 140.0
    stance_height: float = 90.0
    legs: tuple = field(default_factory=tuple)

    def leg(self, name):
        for leg in self.legs:
            if leg.name == name:
                return leg
        raise KeyError(name)

    def neutral_foot(self, leg):
        """Position neutre du pied de `leg` dans le repère corps."""
        return (
            leg.mount_x + self.stance_radius * math.cos(leg.mount_angle),
            leg.mount_y + self.stance_radius * math.sin(leg.mount_angle),
            -self.stance_height,
        )


LEG_NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")


def default_config():
    """Hexapode « classique » : châssis hexagonal, 18 servos sur 2 PCA9685.

    Pattes droites sur la carte 0 (canaux 0-8), gauches sur la carte 1.
    Les servos du côté gauche sont montés en miroir, d'où direction = -1 sur
    le fémur et le tibia.
    """

    def joints(board, base, mirrored):
        d = -1 if mirrored else 1
        return dict(
            coxa=JointServo(board, base, direction=1),
            femur=JointServo(board, base + 1, direction=d),
            # Tibia à la verticale (-90° par rapport au fémur) quand le servo est à 90°.
            tibia=JointServo(board, base + 2, zero_deg=-90.0, direction=d),
        )

    geometry = {
        # nom: (x, y, angle de montage en degrés)
        "RF": (60.0, -40.0, -45.0),
        "RM": (0.0, -60.0, -90.0),
        "RR": (-60.0, -40.0, -135.0),
        "LF": (60.0, 40.0, 45.0),
        "LM": (0.0, 60.0, 90.0),
        "LR": (-60.0, 40.0, 135.0),
    }
    legs = []
    for name in LEG_NAMES:
        x, y, a = geometry[name]
        left = name.startswith("L")
        index = "FMR".index(name[1])
        legs.append(
            LegConfig(
                name=name,
                mount_x=x,
                mount_y=y,
                mount_angle=math.radians(a),
                **joints(1 if left else 0, index * 3, left),
            )
        )
    return HexapodConfig(legs=tuple(legs))
