"""Géométrie du robot et câblage des servos.

Repère du corps : x vers l'avant, y vers la gauche, z vers le haut, origine au
centre du châssis. Toutes les longueurs sont en millimètres, les angles en
radians sauf mention contraire.

La géométrie est celle de l'araignée V1.5 dessinée dans Fusion 360 (document
« hexa », servos MG92B, 2 PCA9685 en 0x40 et 0x41, câblage dans WIRING). Pour
d'autres pièces, modifie seulement DIMENSIONS ci-dessous : tout le reste en
découle.
"""

import math

DIMENSIONS = {
    # Araignée V1.5 (cotes relevées dans Fusion le 2 novembre 2026).
    # Axes des coxas sur un cercle de 78 mm, une patte tous les 60°.
    "mount_middle_y": 78.0,  # pattes du milieu : écart latéral au centre
    "mount_corner_x": 67.55,  # pattes avant/arrière : écart longitudinal
    "mount_corner_y": 39.0,  # pattes avant/arrière : écart latéral
    "corner_angle": 30.0,  # pattes avant/arrière : angle par rapport à l'axe avant/arrière (°)
    "root_to_coxa": 0.0,  # bord du châssis -> axe du servo de coxa (pour l'affichage)
    # Segments d'une patte, d'axe à axe (coxa et fémur à la même hauteur).
    "coxa": 30.0,  # axe coxa -> axe fémur
    "femur": 55.0,  # axe fémur -> axe tibia
    "tibia": 94.5,  # axe tibia -> contact au sol (pied PLA ; 94,8 en TPU)
    # Hauteur debout : axes des fémurs -> sol. Dessous du corps 18,3 mm sous
    # les axes, à 50 mm du sol ; la batterie descend 38,3 mm sous les axes.
    "stand_height": 68.3,
}

# Posture debout : tibia vertical (pied proche du corps, peu de bras de levier
# sur le servo de fémur) ; le fémur se lève de ce qu'il faut pour la hauteur
# demandée (28,5° pour la V1.5).
STAND_TIBIA_OUT_DEG = 0.0

# Repli maximal du tibia vers le corps (°, 0 = perpendiculaire au fémur) selon
# l'angle du fémur (°, + = pied vers le haut) avant de toucher la coxa, la
# flasque ou la batterie (relevé dans Fusion). Vérifié par les tests.
TIBIA_FOLD_LIMITS = ((-90, 6), (-75, 20), (-60, 44), (-45, 51), (-30, 57), (-15, 62),
                     (0, 64), (15, 59), (30, 56), (75, 56), (90, 58))


def tibia_fold_limit(femur_deg):
    """Repli maximal du tibia (°) pour un fémur à `femur_deg`, interpolé."""
    pts = TIBIA_FOLD_LIMITS
    if femur_deg <= pts[0][0]:
        return pts[0][1]
    for (f0, t0), (f1, t1) in zip(pts, pts[1:]):
        if femur_deg <= f1:
            return t0 + (t1 - t0) * (femur_deg - f0) / (f1 - f0)
    return pts[-1][1]


class JointServo:
    """Lien entre un angle articulaire (IK) et un canal de servo PCA9685.

    angle_servo_deg = 90 + direction * (degrés(angle_ik) - zero_deg) + trim_deg

    board : index de la carte PCA9685 (0 = 0x40, 1 = 0x41).
    zero_deg : angle IK (en degrés) qui place le servo au milieu (90°).
    direction : +1 ou -1 selon le sens de montage (inversé si `inverted`).
    trim_deg : réglage fin mesuré sur le robot réel (calibration).
    min_deg / max_deg : butées, en degrés servo.
    """

    def __init__(self, board, channel, zero_deg=0.0, direction=1, trim_deg=0.0,
                 min_deg=30.0, max_deg=150.0):
        self.board = board
        self.channel = channel
        self.zero_deg = zero_deg
        self.default_direction = direction
        self.direction = direction
        self.trim_deg = trim_deg
        self.min_deg = min_deg
        self.max_deg = max_deg


    @property
    def inverted(self):
        return self.direction != self.default_direction

    @inverted.setter
    def inverted(self, value):
        # Servo monté dans l'autre sens que prévu (réglé à la calibration).
        self.direction = -self.default_direction if value else self.default_direction


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
        self.sit_drop = 0.25 * stance_height if sit_drop is None else sit_drop
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

# Câblage : (carte, premier canal) ; coxa, fémur, tibia sur trois canaux
# consécutifs. Carte 0 = 0x40 : pattes avant et milieu gauche ;
# carte 1 = 0x41 (pont A0 soudé) : milieu droite et pattes arrière.
WIRING = {
    "RF": (0, 5),
    "LF": (0, 2),
    "LM": (0, 8),
    "RM": (1, 2),
    "RR": (1, 8),
    "LR": (1, 5),
}


def default_config(dimensions=None):
    d = dict(DIMENSIONS)
    d.update(dimensions or {})

    ca = d["corner_angle"]
    mounts = {
        # nom : (x, y, angle en degrés) du point de fixation sur le châssis
        "RF": (d["mount_corner_x"], -d["mount_corner_y"], -ca),
        "RM": (0.0, -d["mount_middle_y"], -90.0),
        "RR": (-d["mount_corner_x"], -d["mount_corner_y"], ca - 180.0),
        "LF": (d["mount_corner_x"], d["mount_corner_y"], ca),
        "LM": (0.0, d["mount_middle_y"], 90.0),
        "LR": (-d["mount_corner_x"], d["mount_corner_y"], 180.0 - ca),
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
                # Les 6 pattes sont la même pièce, tournée (pas de symétrie
                # gauche/droite). Servos au milieu : coxa dans l'axe de la
                # patte, fémur à plat, tibia perpendiculaire au fémur. Vus
                # côté axe, coxa + = sens trigo, fémur + (pied vers le haut)
                # = sens horaire, tibia + (pied vers le corps) = sens trigo.
                # Sens à confirmer sur le robot (bouton « Inverser le sens »).
                # Coxa : ±60°, le corps l'arrête à ±64°.
                coxa=JointServo(board, ch, min_deg=30.0, max_deg=150.0),
                femur=JointServo(board, ch + 1, zero_deg=0.0, direction=-1),
                # Tibia : butée sur la coxa à +56° vers le corps.
                tibia=JointServo(board, ch + 2, zero_deg=-90.0, direction=-1, max_deg=145.0),
            )
        )

    tibia_a = STAND_TIBIA_OUT_DEG * math.pi / 180.0
    stance_height = d["stand_height"]
    femur_a = math.asin((d["tibia"] * math.cos(tibia_a) - stance_height) / d["femur"])
    stance_radius = d["coxa"] + d["femur"] * math.cos(femur_a) + d["tibia"] * math.sin(tibia_a)
    return HexapodConfig(
        legs=legs,
        coxa_length=d["coxa"],
        femur_length=d["femur"],
        tibia_length=d["tibia"],
        stance_radius=stance_radius,
        stance_height=stance_height,
        # Longueur de pas et levée proportionnelles à la taille de la patte.
        # Pas jusqu'à 0,55 tibia (52 mm), pied levé de 19 mm : reste dans la
        # course des servos sans que le tibia touche la coxa
        # (vérifié par les tests, dans toutes les directions).
        step_height=0.2 * d["tibia"],
        max_stride=0.55 * d["tibia"],
    )
