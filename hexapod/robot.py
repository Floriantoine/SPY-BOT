"""Contrôleur haut niveau : démarche + pose du corps -> IK -> servos."""

import math

from .body_ik import BodyPose, body_to_leg_frame, foot_in_body, leg_to_body_frame
from .gait import QUAD_TUCKED, GaitCommand, QuadGait, TripodGait
from .leg_ik import LegAngles, leg_fk, leg_ik
from .servo import DryRunDriver, ik_to_servo_deg

TUCK_TIME = 1.0  # s pour replier ou déplier les pattes du milieu
# Patte repliée : fémur relevé de 65°, tibia replié (pied ~6 cm au-dessus du
# sol debout, ~3 cm assis), dans la course des servos.
TUCK_ANGLES = LegAngles(0.0, math.radians(65), math.radians(-110))


class Hexapod:
    def __init__(self, config, driver=None, gait=None):
        self.config = config
        self.driver = driver or DryRunDriver()
        self.tripod = gait or TripodGait(step_height=config.step_height, max_stride=config.max_stride)
        neutral = {leg.name: config.neutral_foot(leg) for leg in config.legs}
        # À 4 pattes, le corps se balance (sway) en plus du pas : pas plus
        # courts pour rester dans la course des servos (vérifié par les tests).
        self.quad = QuadGait(neutral, period=2 * self.tripod.period, step_height=config.step_height,
                             max_stride=min(config.max_stride, 0.52 * config.tibia_length),
                             sway=0.22 * config.tibia_length)
        self.gait = self.tripod
        self.legs = 6  # pattes qui marchent : 6, ou 4 (pattes du milieu repliées)
        self.tuck = 0.0  # 0 = pattes du milieu au sol, 1 = repliées
        self.command = GaitCommand()
        self.pose = BodyPose()

    def set_legs(self, n):
        """Passe en marche à 6 ou 4 pattes. Seulement à l'arrêt, pieds au sol."""
        if n not in (4, 6):
            raise ValueError("4 ou 6 pattes")
        if n == self.legs:
            return
        if not (self.command.is_zero and self.gait.at_rest):
            raise ValueError("arrêter le robot d'abord")
        self.legs = n
        self.gait = self.quad if n == 4 else self.tripod
        self.gait.reset()

    @property
    def tuck_settled(self):
        return self.tuck == (1.0 if self.legs == 4 else 0.0)

    def foot_targets(self):
        """Position cible de chaque pied dans le repère corps neutre."""
        targets = {}
        for leg in self.config.legs:
            neutral = self.config.neutral_foot(leg)
            dx, dy, dz = self.gait.foot_offset(leg.name, neutral, self.command)
            targets[leg.name] = (neutral[0] + dx, neutral[1] + dy, neutral[2] + dz)
        return targets

    def tucked_foot(self, leg):
        """Pied replié, dans le repère du corps (il suit le corps, pas le sol)."""
        c = self.config
        foot = leg_fk(TUCK_ANGLES, c.coxa_length, c.femur_length, c.tibia_length)
        return leg_to_body_frame(foot, leg)

    def solve(self):
        """Angles articulaires de chaque patte pour l'état courant.

        Lève UnreachableError si une patte ne peut pas atteindre sa cible.
        """
        c = self.config
        targets = self.foot_targets()
        angles = {}
        for leg in c.legs:
            foot = foot_in_body(targets[leg.name], self.pose)
            if leg.name in QUAD_TUCKED and self.tuck > 0.0:
                t = self.tucked_foot(leg)
                s = self.tuck * self.tuck * (3 - 2 * self.tuck)
                foot = tuple(a + (b - a) * s for a, b in zip(foot, t))
            x, y, z = body_to_leg_frame(foot, leg)
            angles[leg.name] = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
        return angles

    def update(self, dt):
        """Avance la démarche de `dt` secondes et envoie les angles aux servos."""
        goal = 1.0 if self.legs == 4 else 0.0
        step = dt / TUCK_TIME
        self.tuck = min(goal, self.tuck + step) if goal > self.tuck else max(goal, self.tuck - step)
        # Pas de marche tant que les pattes du milieu bougent.
        self.gait.advance(dt, self.command if self.tuck_settled else GaitCommand())
        angles = self.solve()
        for leg in self.config.legs:
            a = angles[leg.name]
            for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                self.driver.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))
        return angles
