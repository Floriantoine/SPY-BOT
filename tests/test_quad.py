import math
import unittest

from hexapod import BodyPose, GaitCommand, default_config
from hexapod.gait import QUAD_ORDER, QUAD_TUCKED
from hexapod.robot import Hexapod

COMMANDS = ((1000, 0, 0), (-1000, 0, 0), (0, 1000, 0), (0, -1000, 0), (700, 700, 0), (0, 0, 3.0), (0, 0, -3.0))


def servo_margin(config, angles, legs=None):
    """Plus petite marge (degrés) entre un servo et sa butée."""
    worst = 180.0
    for leg in config.legs:
        if legs is not None and leg.name not in legs:
            continue
        a = angles[leg.name]
        for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
            raw = 90.0 + joint.direction * (math.degrees(value) - joint.zero_deg)
            worst = min(worst, raw - joint.min_deg, joint.max_deg - raw)
    return worst


def margin_inside(point, triangle):
    """Distance signée de `point` au bord du triangle (positive à l'intérieur)."""
    (ax, ay), (bx, by), (cx, cy) = triangle
    orient = 1.0 if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) > 0 else -1.0
    best = 1e9
    for (x1, y1), (x2, y2) in (((ax, ay), (bx, by)), ((bx, by), (cx, cy)), ((cx, cy), (ax, ay))):
        n = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        cross = (x2 - x1) * (point[1] - y1) - (y2 - y1) * (point[0] - x1)
        best = min(best, orient * cross / n)
    return best


class QuadWalkTest(unittest.TestCase):
    def setUp(self):
        self.config = default_config()

    def walking(self, command, steps=600):
        robot = Hexapod(self.config)
        robot.set_legs(4)
        robot.command = GaitCommand(*command)
        for _ in range(steps):
            yield robot, robot.update(0.02)

    def test_tucked_legs_fold_up_and_stay_in_the_air(self):
        for robot, _ in self.walking((0, 0, 0), 60):
            pass
        self.assertEqual(robot.tuck, 1.0)
        targets = robot.foot_targets()
        for name in QUAD_TUCKED:
            foot = robot.tucked_foot(self.config.leg(name))
            self.assertGreater(foot[2], targets["RF"][2] + 40)  # bien au-dessus du sol

    def test_one_corner_leg_in_the_air_at_a_time(self):
        for robot, _ in self.walking((150, 0, 0)):
            in_air = [n for n in QUAD_ORDER if n not in robot.gait.feet_on_ground()]
            self.assertLessEqual(len(in_air), 1)

    def test_body_stays_inside_the_three_supporting_feet(self):
        # Le centre du corps (origine du repère corps) doit rester à au moins
        # 12 mm du bord du triangle d'appui quand une patte est levée.
        worst = 1e9
        for command in COMMANDS:
            for robot, _ in self.walking(command):
                grounded = robot.gait.feet_on_ground()
                if len(grounded) == 3:
                    targets = robot.foot_targets()
                    worst = min(worst, margin_inside((0.0, 0.0), [targets[n][:2] for n in grounded]))
        self.assertGreater(worst, 12.0)

    def test_servos_stay_inside_their_range(self):
        # Pieds au sol : position exacte, dans la course des servos. Un pied en
        # l'air peut toucher une butée au sommet du pas (il passe alors un peu
        # plus bas), sans conséquence.
        for command in COMMANDS:
            for robot, angles in self.walking(command):
                on_ground = [n for n in robot.neutral if robot.gait.swing(n) is None]
                self.assertGreater(servo_margin(self.config, angles, on_ground), 3.0, command)
        for height in (0.0, -self.config.sit_drop):
            robot = Hexapod(self.config)
            robot.set_legs(4)
            robot.tuck = 1.0
            robot.pose = BodyPose(0, 0, height)
            self.assertGreater(servo_margin(self.config, robot.solve()), 3.0)

    def test_switch_only_when_stopped(self):
        robot = Hexapod(self.config)
        robot.command = GaitCommand(100, 0, 0)
        robot.update(0.1)
        with self.assertRaises(ValueError):
            robot.set_legs(4)
        robot.command = GaitCommand()
        for _ in range(100):
            robot.update(0.02)
        robot.set_legs(4)
        self.assertEqual(robot.legs, 4)

    def test_does_not_walk_while_folding(self):
        robot = Hexapod(self.config)
        robot.set_legs(4)
        robot.command = GaitCommand(100, 0, 0)
        robot.update(0.5)
        self.assertEqual(robot.gait.phase, 0.0)
        for _ in range(40):
            robot.update(0.02)
        self.assertNotEqual(robot.gait.phase, 0.0)

    def test_stops_with_all_feet_down_and_body_centred(self):
        robot = Hexapod(self.config)
        robot.set_legs(4)
        robot.command = GaitCommand(100, 0, 0)
        for _ in range(130):
            robot.update(0.02)
        robot.command = GaitCommand()
        for _ in range(200):
            robot.update(0.02)
        self.assertTrue(robot.gait.at_rest)
        self.assertEqual(len(robot.gait.feet_on_ground()), 4)
        self.assertLess(abs(robot.gait.shift[0]) + abs(robot.gait.shift[1]), 1e-6)


if __name__ == "__main__":
    unittest.main()
