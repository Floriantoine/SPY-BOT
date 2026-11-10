"""Marche plus douce : poser du pied, démarches, décalage du corps, servos."""

import math
import unittest

from hexapod import BodyPose, GaitCommand, default_config
from hexapod.config import MIDDLE_LEGS, tibia_fold_limit
from hexapod.robot import WAVE_AUTO, WAVE_OFF, WAVE_ON, Hexapod, angles_ok
from hexapod.servo import ik_to_servo_deg

DT = 0.02
# Entre deux ticks : un pied posé suit le sol (vitesse du corps), un pied en
# l'air va au plus à ~0,5 m/s (au poser, il rejoint son point de pose, qui
# suit la consigne : quelques mm de plus pendant un freinage).
STANCE_JUMP = 4.0  # mm
SWING_JUMP = 12.0  # mm
SWAY_JUMP = 8.0  # mm : à 4 pattes, le corps se décale en plus


def walk(robot, command, seconds, watch=None):
    robot.command = command
    for _ in range(int(round(seconds / DT))):
        robot.update(DT)
        if watch:
            watch()


class Watch:
    """Vérifie à chaque tick ce qu'une marche doit toujours respecter :
    les pieds posés ne glissent pas (ils bougent tous ensemble, vus du
    corps), aucune cible de pied ne saute, aucun servo ne dépasse sa course
    et aucune pièce n'en touche une autre."""

    def __init__(self, test, robot, where=""):
        self.t, self.r, self.where = test, robot, where
        self.before = robot.foot_targets()
        self.grounded = self._grounded()
        self.gaits = [robot.gait]
        self.worst_slip = self.worst_jump = 0.0
        self.min_down = 6

    def _grounded(self):
        r = self.r
        return set(n for n in r.walking_legs() if r.offsets[n][2] == 0.0 and r.gait.swing(n) in (None, 0.0))

    def __call__(self):
        t, r = self.t, self.r
        after = r.foot_targets()
        now = self._grounded()
        self.min_down = min(self.min_down, len(now))
        both = sorted(self.grounded & now)
        for i in range(len(both)):
            for j in range(i + 1, len(both)):
                d0 = math.dist(self.before[both[i]][:2], self.before[both[j]][:2])
                d1 = math.dist(after[both[i]][:2], after[both[j]][:2])
                self.worst_slip = max(self.worst_slip, abs(d0 - d1))
                t.assertAlmostEqual(d0, d1, delta=0.05, msg=(self.where, both[i], both[j]))
        for n in r.walking_legs():
            jump = math.dist(self.before[n], after[n])
            self.worst_jump = max(self.worst_jump, jump)
            limit = SWING_JUMP if n not in both else SWAY_JUMP if r.legs == 4 else STANCE_JUMP
            t.assertLess(jump, limit, (self.where, n))
        angles = r._written
        t.assertTrue(angles_ok(r.config, angles), self.where)
        for leg in r.config.legs:
            a = angles[leg.name]
            for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                d = 90.0 + joint.direction * (math.degrees(v) - joint.zero_deg) + joint.trim_deg
                t.assertEqual(ik_to_servo_deg(v, joint), d, (self.where, leg.name))  # pas en butée
            fold = -math.degrees(a.tibia) - 90.0
            t.assertLess(fold, tibia_fold_limit(math.degrees(a.femur), leg.name in MIDDLE_LEGS), (self.where, leg.name))
        if r.gait is not self.gaits[-1]:
            self.gaits.append(r.gait)
        self.before, self.grounded = after, now


COMMANDS = ((15, 0, 0), (100, 0, 0), (-100, 0, 0), (0, 100, 0), (0, -30, 0), (70, 70, 0), (0, 0, 0.1), (0, 0, 1.0),
            (0, 0, -1.0), (40, 0, 0.3), (-60, 40, -0.5), (10, 5, 0.05), (0, 0, 0))


class WaveWalkTest(unittest.TestCase):
    def test_five_feet_down_no_slip_inside_limits(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        w = Watch(self, r, "vague")
        for c in COMMANDS:
            w.where = ("vague", c)
            walk(r, GaitCommand(*c), 6, w)
        self.assertIs(r.gait, r.wave)
        self.assertEqual(w.min_down, 5)  # jamais plus d'une patte en l'air
        self.assertTrue(r.at_rest)
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))  # sans le filet de sécurité

    def test_slower_than_tripod_but_walks(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        walk(r, GaitCommand(100.0, 0, 0), 6)
        dist = [0.0]

        def watch():
            dist[0] += r.travel[0]

        walk(r, GaitCommand(100.0, 0, 0), 10, watch)
        self.assertGreater(dist[0] / 10, 20.0)  # mm/s
        self.assertLess(dist[0] / 10, 50.0)

    def test_switch_while_walking(self):
        """Tripode <-> vague en pleine marche, à tous les moments du cycle."""
        for delay in range(0, 75, 7):
            for c in ((25, 0, 0), (0, 20, 0.1), (100, 30, -0.3), (0, 0, 0.5)):
                r = Hexapod(default_config())
                w = Watch(self, r, (delay, c))
                walk(r, GaitCommand(*c), 3 + delay * DT, w)
                r.wave_mode = WAVE_ON
                walk(r, GaitCommand(*c), 4 + delay * DT, w)
                self.assertIs(r.gait, r.wave, (delay, c))
                r.wave_mode = WAVE_OFF
                walk(r, GaitCommand(*c), 3, w)
                self.assertIs(r.gait, r.tripod, (delay, c))
                walk(r, GaitCommand(), 4, w)
                self.assertTrue(r.at_rest, (delay, c))

    def test_auto_follows_the_speed(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_AUTO
        w = Watch(self, r, "auto")
        walk(r, GaitCommand(10.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(60.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(18.0, 0, 0), 5, w)  # entre 80 % du seuil et le seuil : on ne rechange pas
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(12.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(18.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(0, 0, 0.5), 5, w)  # tourner vite : les pieds vont vite
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(), 4, w)
        self.assertIs(r.gait, r.tripod)  # à l'arrêt : pas de changement
        self.assertTrue(r.at_rest)
        r.command = GaitCommand(5.0, 0, 0)
        r.update(DT)
        self.assertIs(r.gait, r.wave)  # départ lent : tout de suite en vague
        for c in COMMANDS:
            w.where = ("auto", c)
            walk(r, GaitCommand(*c), 5, w)
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))

    def test_default_is_tripod(self):
        r = Hexapod(default_config())
        walk(r, GaitCommand(5.0, 0, 0), 3)
        self.assertIs(r.gait, r.tripod)

    def test_four_legs_keep_their_gait(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        walk(r, GaitCommand(20.0, 0, 0), 3)
        walk(r, GaitCommand(), 4)
        r.set_legs(4)
        w = Watch(self, r, "4 pattes")
        walk(r, GaitCommand(), 1.5, w)
        walk(r, GaitCommand(50.0, 0, 0.2), 6, w)
        self.assertIs(r.gait, r.quad)
        walk(r, GaitCommand(), 6, w)
        r.set_legs(6)
        walk(r, GaitCommand(), 1.5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(20.0, 0, 0), 4, w)
        self.assertEqual(w.min_down, 3)  # à 4 pattes : 3 pieds posés

    def test_strong_pose_stays_inside_limits(self):
        """Le filet de sécurité (pose et levée réduites) marche aussi en vague."""
        for mode in (WAVE_ON, WAVE_AUTO):
            r = Hexapod(default_config())
            r.wave_mode = mode
            r.pose = BodyPose(20, 20, -15, math.radians(8), math.radians(8), math.radians(20))
            w = Watch(self, r, ("pose", mode))
            kept = [1.0]

            def watch():
                w()
                kept[0] = min(kept[0], r.pose_scale)

            for c in ((30, 10, 0.1), (100, 0, 0), (10, 0, 0), (0, 0, 0)):
                walk(r, GaitCommand(*c), 5, watch)
            self.assertLess(kept[0], 1.0)


class SoftTouchdownTest(unittest.TestCase):
    def check(self, legs):
        r = Hexapod(default_config())
        if legs == 4:
            r.set_legs(4)
            walk(r, GaitCommand(), 1.5)
        h = r.gait.step_height
        prev = {}
        seen = {"peak": 0.0, "step": 0.0, "landings": 0}

        def watch():
            for n in r.walking_legs():
                z = r.offsets[n][2]
                was = prev.get(n, 0.0)
                if (z == 0.0) != (was == 0.0):
                    # Décollage ou poser : presque sans vitesse verticale.
                    seen["step"] = max(seen["step"], abs(z - was))
                    seen["landings"] += z == 0.0
                seen["peak"] = max(seen["peak"], z)
                prev[n] = z

        walk(r, GaitCommand(60.0, 20.0, 0.2), 12, watch)
        self.assertGreater(seen["landings"], 10)
        self.assertGreater(seen["peak"], 0.97 * h)  # même hauteur qu'avant au milieu du pas
        self.assertLessEqual(seen["peak"], h + 1e-9)
        # Avec sin(pi s), le dernier tick descendait de ~13 % de la hauteur (6 pattes).
        self.assertLess(seen["step"], 0.03 * h)

    def test_six_legs(self):
        self.check(6)

    def test_four_legs(self):
        self.check(4)


if __name__ == "__main__":
    unittest.main()
