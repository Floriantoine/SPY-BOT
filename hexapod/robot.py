"""Contrôleur haut niveau : démarche + pose du corps -> IK -> servos."""

from .body_ik import BodyPose, body_to_leg_frame, foot_in_body
from .gait import GaitCommand, TripodGait
from .leg_ik import leg_ik
from .servo import DryRunDriver, ik_to_servo_deg


class Hexapod:
    def __init__(self, config, driver=None, gait=None):
        self.config = config
        self.driver = driver or DryRunDriver()
        self.gait = gait or TripodGait()
        self.command = GaitCommand()
        self.pose = BodyPose()

    def foot_targets(self):
        """Position cible de chaque pied dans le repère corps neutre."""
        targets = {}
        for leg in self.config.legs:
            neutral = self.config.neutral_foot(leg)
            dx, dy, dz = self.gait.foot_offset(leg.name, neutral, self.command)
            targets[leg.name] = (neutral[0] + dx, neutral[1] + dy, neutral[2] + dz)
        return targets

    def solve(self):
        """Angles articulaires de chaque patte pour l'état courant.

        Lève UnreachableError si une patte ne peut pas atteindre sa cible.
        """
        c = self.config
        targets = self.foot_targets()
        angles = {}
        for leg in c.legs:
            foot = foot_in_body(targets[leg.name], self.pose)
            x, y, z = body_to_leg_frame(foot, leg)
            angles[leg.name] = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
        return angles

    def update(self, dt):
        """Avance la démarche de `dt` secondes et envoie les angles aux servos."""
        self.gait.advance(dt, self.command)
        angles = self.solve()
        for leg in self.config.legs:
            a = angles[leg.name]
            for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                self.driver.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))
        return angles
