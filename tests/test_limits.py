"""Le robot ne met jamais un servo en butée : une pose du corps trop forte
pour marcher est réduite (et les pas moins hauts), puis revient."""

import math
import unittest

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.imu import SimIMU
from hexapod.body_ik import BodyPose
from hexapod.robot import angles_ok
from hexapod.sensor import SimRanger
from hexapod.servo import DryRunDriver


def make():
    c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", SimRanger([]), SimIMU())
    c.handle("POWER 1")
    c.handle("STAND")
    run(c, 3.0)
    return c


def run(c, seconds, move=None, watch=None):
    lines = []
    for i in range(int(seconds / 0.02)):
        if move and i % 25 == 0:
            c.handle(move)
        out = c.tick(0.02)
        if out:
            lines += out.split("\n")
        if watch:
            watch(c)
    return lines


def servo_margin(c):
    """Plus petit écart (°) entre un servo et sa butée, comme l'appli le calcule."""
    worst = 180.0
    for leg in c.robot.config.legs:
        a = c.angles[leg.name]
        for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
            d = 90.0 + joint.direction * (math.degrees(v) - joint.zero_deg) + joint.trim_deg
            worst = min(worst, d - joint.min_deg, joint.max_deg - d)
    return worst


class LimitTest(unittest.TestCase):
    POSES = ("POSE 0 0 -15 0 0 0", "POSE 20 20 0 0 0 0", "POSE 0 0 0 8 8 0",
             "POSE 20 20 -15 8 8 20", "POSE -20 -20 15 -8 -8 -20")
    MOVES = ("MOVE 100 0 30", "MOVE 0 100 0", "MOVE -70 70 -60")

    def test_never_at_a_servo_limit(self):
        for pose in self.POSES:
            for move in self.MOVES:
                c = make()
                c.handle(pose)
                seen = []
                run(c, 4.0, move, lambda c: seen.append(servo_margin(c)))
                self.assertGreater(min(seen), 0.5, (pose, move))
                self.assertTrue(angles_ok(c.robot.config, c.angles), (pose, move))

    def test_leveling_on_a_slope_while_walking(self):
        c = make()
        c.handle("TILT 12 12")
        c.handle("LEVEL 1")
        seen = []
        run(c, 6.0, "MOVE 100 0 0", lambda c: seen.append(servo_margin(c)))
        self.assertGreater(min(seen), 0.5)

    def test_plain_walking_is_untouched(self):
        c = make()
        run(c, 4.0, "MOVE 100 0 30", lambda c: self.assertEqual((c.robot.pose_scale, c.robot.lift_scale), (1.0, 1.0)))

    def test_pose_kept_when_possible_and_restored(self):
        c = make()
        c.handle("POSE 0 0 -15 0 0 0")
        run(c, 2.0)
        self.assertEqual(c.robot.pose_scale, 1.0)  # immobile : la pose tient
        lines = run(c, 4.0, "MOVE 100 0 30")
        # Pieds posés plus loin du corps : la pose et la hauteur de pas tiennent.
        self.assertEqual((c.robot.pose_scale, c.robot.lift_scale), (1.0, 1.0))
        c.handle("TILT 12 12")
        c.handle("LEVEL 1")
        lines += run(c, 6.0, "MOVE 100 0 30")
        self.assertIn("LIMIT pose 1", lines)
        c.handle("MOVE 0 0 0")
        c.handle("LEVEL 0")
        c.handle("POSE 0 0 0 5 0 0")  # tient à l'arrêt
        lines = run(c, 5.0)
        self.assertIn("LIMIT pose 0", lines)
        self.assertEqual(c.robot.pose_scale, 1.0)
        self.assertAlmostEqual(math.degrees(c.robot.applied.roll), 5.0, places=3)



class FootPlacementTest(unittest.TestCase):
    def test_feet_move_out_when_body_lowered(self):
        c = make()
        c.handle("POSE 0 0 -15 0 0 0")
        run(c, 4.0)  # à l'arrêt : un pas sur place pour replacer les pieds
        self.assertTrue(c.robot.at_rest)
        for n, (hx, hy) in c.robot.home.items():
            self.assertGreater(math.hypot(hx, hy), 20.0, n)
            o = c.robot.offsets[n]
            self.assertAlmostEqual(o[0], hx, delta=0.5)
            self.assertAlmostEqual(o[1], hy, delta=0.5)
        phase = c.robot.gait.phase
        run(c, 2.0)
        self.assertEqual(c.robot.gait.phase, phase)  # puis il ne bouge plus

    def test_neutral_pose_keeps_neutral_stance(self):
        c = make()
        run(c, 3.0, "MOVE 100 0 30")
        self.assertEqual(set(c.robot.home.values()), {(0.0, 0.0)})

    def test_no_stepping_while_standing_up_or_sitting(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", SimRanger([]), SimIMU())
        c.handle("POWER 1")
        c.handle("POSE 0 0 -15 0 0 0")
        c.handle("STAND")
        while c.mode != "stand":
            self.assertEqual(c.robot.gait.phase, 0.0)  # pas de pas en se levant
            c.tick(0.02)
        run(c, 4.0)
        c.handle("POSE 0 0 0 0 0 0")
        c.handle("SIT")
        run(c, 6.0)
        self.assertEqual(c.mode, "sit")


if __name__ == "__main__":
    unittest.main()


class QuadSwitchTest(unittest.TestCase):
    def test_lowered_body_can_switch_back_to_six_legs(self):
        from hexapod import Hexapod, default_config as cfg
        r = Hexapod(cfg())
        r.pose = BodyPose(z=-10)
        for _ in range(400):
            r.update(0.02)
        self.assertTrue(r.at_rest)
        r.set_legs(4)
        for _ in range(500):
            r.update(0.02)
        self.assertTrue(r.at_rest)  # les pattes repliées ne le font pas piétiner
        r.set_legs(6)

    def test_no_jump_when_switching_after_walking_on_four(self):
        from hexapod import GaitCommand, Hexapod, default_config as cfg
        from hexapod.servo import DryRunDriver
        r = Hexapod(cfg(), DryRunDriver())
        r.set_legs(4)
        for _ in range(100):
            r.update(0.02)
        r.command = GaitCommand(60, 0, 0)
        for _ in range(300):
            r.update(0.02)
        r.command = GaitCommand()
        for _ in range(1000):
            r.update(0.02)
            if r.at_rest:
                break
        before = dict(r.driver.angles)
        r.set_legs(6)
        r.update(0.02)
        jump = max(abs(v - before[k]) for k, v in r.driver.angles.items())
        self.assertLess(jump, 3.0)  # degrés : le corps était revenu au centre
