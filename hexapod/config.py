"""Géométrie du robot et câblage des servos.

Repère du corps : x vers l'avant, y vers la gauche, z vers le haut, origine au
centre du châssis. Toutes les longueurs sont en millimètres, les angles en
radians sauf mention contraire.

La géométrie reprend le hexapode SmallpTsai/hexapod-v2-7697 (servos MG92B,
2 PCA9685 en 0x40 et 0x41), pièces agrandies à 150 %. Pour d'autres pièces,
modifie seulement DIMENSIONS ci-dessous : tout le reste en découle.
"""

import math

DIMENSIONS = {
    # Robot de l'utilisateur : pièces de hexapod-v2-7697 imprimées à 150 %.
    # Les valeurs mesurées sur le robot remplacent celles calculées
    # (origine × 1,5 entre parenthèses).
    # Fixation des pattes sur le châssis.
    "mount_middle_y": 43.85,  # pattes du milieu : écart latéral au centre (mesuré : corps de 87,7 de large)
    "mount_corner_x": 83.12,  # pattes avant/arrière : écart longitudinal (55,41 × 1,5)
    "mount_corner_y": 33.62,  # pattes avant/arrière : écart latéral (22,41 × 1,5)
    # Segments d'une patte.
    "root_to_coxa": 31.13,  # fixation châssis -> axe du servo de coxa (20,75 × 1,5)
    "coxa": 45.0,  # axe coxa -> axe fémur (mesuré, 42 calculé)
    "femur": 59.0,  # axe fémur -> axe tibia (mesuré, 63,9 calculé)
    "tibia": 135.0,  # axe tibia -> bout du pied (mesuré, 133,6 calculé)
}

# Posture debout : fémur relevé de 30°, tibia incliné de 15° vers l'extérieur
# (posture « standby » de hexapod-v2-7697).
STAND_FEMUR_DEG = 30.0
STAND_TIBIA_OUT_DEG = 15.0


class JointServo:
    """Lien entre un angle articulaire (IK) et un canal de servo PCA9685.

    angle_servo_deg = 90 + direction * (degrés(angle_ik) - zero_deg) + trim_deg

    board : index de la carte PCA9685 (0 = 0x40, 1 = 0x41).
    zero_deg : angle IK (en degrés) qui place le servo au milieu (90°).
    direction : +1 ou -1 selon le sens de montage.
    trim_deg : réglage fin mesuré sur le robot réel (calibration).
    min_deg / max_deg : butées, en degrés servo.
    """

    def __init__(self, board, channel, zero_deg=0.0, direction=1, trim_deg=0.0,
                 min_deg=30.0, max_deg=150.0):
        self.board = board
        self.channel = channel
        self.zero_deg = zero_deg
        self.direction = direction
        self.trim_deg = trim_deg
        self.min_deg = min_deg
        self.max_deg = max_deg


class LegConfig:
    """Une patte : position (mm) de l'axe coxa dans le repère corps, son
    orientation (rad, 0 = vers l'avant), le point de fixation sur le châssis
    (pour l'affichage) et ses trois servos."""

    def __init__(self, name, mount_x, mount_y, mount_angle, coxa, femur, tibia,
                 body_x=None, body_y=None):
        self.name = name
        self.mount_x = mount_x
        self.mount_y = mount_y
        self.mount_angle = mount_angle
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia
        self.body_x = mount_x if body_x is None else body_x
        self.body_y = mount_y if body_y is None else body_y


class HexapodConfig:
    """Dimensions du robot. stance_radius et stance_height donnent la position
    neutre des pieds : distance horizontale depuis l'axe coxa et hauteur sous
    le corps. sit_drop : de combien le corps descend en position assise."""

    def __init__(self, legs=(), coxa_length=50.0, femur_length=80.0, tibia_length=120.0,
                 stance_radius=140.0, stance_height=90.0, sit_drop=None,
                 step_height=30.0, max_stride=60.0):
        self.legs = tuple(legs)
        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length
        self.stance_radius = stance_radius
        self.stance_height = stance_height
        # Limité pour que le fémur reste dans la course de son servo (~60° de levée).
        self.sit_drop = 0.3 * stance_height if sit_drop is None else sit_drop
        self.step_height = step_height
        self.max_stride = max_stride

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

    def to_dict(self):
        """Géométrie sérialisable (pour l'interface PC)."""
        return {
            "coxa": self.coxa_length,
            "femur": self.femur_length,
            "tibia": self.tibia_length,
            "addresses": list(I2C_ADDRESSES),
            "legs": [
                {
                    "name": leg.name,
                    "x": leg.mount_x,
                    "y": leg.mount_y,
                    "angle": leg.mount_angle,
                    "body_x": leg.body_x,
                    "body_y": leg.body_y,
                    "servos": [[j.board, j.channel] for j in (leg.coxa, leg.femur, leg.tibia)],
                    # Pour l'interface : angle servo = 90 + sens × (angle - zéro) + trim,
                    # borné entre min et max (sert à montrer les servos en butée).
                    "joints": [[j.zero_deg, j.direction, j.min_deg, j.max_deg] for j in (leg.coxa, leg.femur, leg.tibia)],
                }
                for leg in self.legs
            ],
        }


LEG_NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")

# Adresse I²C de chaque carte PCA9685 (index = « carte » dans WIRING).
I2C_ADDRESSES = (0x40, 0x41)

# Câblage de hexapod-v2-7697 : (carte, premier canal) ; coxa, fémur, tibia
# sur trois canaux consécutifs. Carte 0 = 0x40 (droite), 1 = 0x41 (gauche).
WIRING = {
    "RF": (0, 5),
    "RM": (0, 2),
    "RR": (0, 8),
    "LF": (1, 5),
    "LM": (1, 2),
    "LR": (1, 8),
}


def default_config(dimensions=None):
    d = dict(DIMENSIONS)
    d.update(dimensions or {})

    mounts = {
        # nom : (x, y, angle en degrés) du point de fixation sur le châssis
        "RF": (d["mount_corner_x"], -d["mount_corner_y"], -45.0),
        "RM": (0.0, -d["mount_middle_y"], -90.0),
        "RR": (-d["mount_corner_x"], -d["mount_corner_y"], -135.0),
        "LF": (d["mount_corner_x"], d["mount_corner_y"], 45.0),
        "LM": (0.0, d["mount_middle_y"], 90.0),
        "LR": (-d["mount_corner_x"], d["mount_corner_y"], 135.0),
    }
    legs = []
    for name in LEG_NAMES:
        x, y, a = mounts[name]
        a = a * math.pi / 180.0
        board, ch = WIRING[name]
        legs.append(
            LegConfig(
                name=name,
                # L'axe de la coxa est décalé de root_to_coxa vers l'extérieur.
                mount_x=x + d["root_to_coxa"] * math.cos(a),
                mount_y=y + d["root_to_coxa"] * math.sin(a),
                mount_angle=a,
                body_x=x,
                body_y=y,
                coxa=JointServo(board, ch, min_deg=45.0, max_deg=135.0),
                # Servo de fémur monté à l'envers, milieu = fémur relevé de 15°.
                femur=JointServo(board, ch + 1, zero_deg=15.0, direction=-1),
                # Milieu = tibia perpendiculaire au fémur.
                tibia=JointServo(board, ch + 2, zero_deg=-90.0),
            )
        )

    femur_a = STAND_FEMUR_DEG * math.pi / 180.0
    tibia_a = STAND_TIBIA_OUT_DEG * math.pi / 180.0
    stance_radius = d["coxa"] + d["femur"] * math.cos(femur_a) + d["tibia"] * math.sin(tibia_a)
    stance_height = d["tibia"] * math.cos(tibia_a) - d["femur"] * math.sin(femur_a)
    return HexapodConfig(
        legs=legs,
        coxa_length=d["coxa"],
        femur_length=d["femur"],
        tibia_length=d["tibia"],
        stance_radius=stance_radius,
        stance_height=stance_height,
        # Longueur de pas et levée proportionnelles à la taille de la patte
        # (25 mm de rayon de pas et de levée sur le robot d'origine).
        step_height=0.28 * d["tibia"],
        max_stride=0.56 * d["tibia"],
    )
