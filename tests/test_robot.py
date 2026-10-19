import math
import unittest

from hexapod import BodyPose, GaitCommand, Hexapod, default_config
from hexapod.body_ik import leg_to_body_frame
from hexapod.leg_ik import leg_fk
from hexapod.servo import ik_to_servo_deg


class HexapodTest(unittest.TestCase):
    def setUp(self):
        self.robot = Hexapod(default_config())

    def test_neutral_pose_puts_servos_near_center(self):
        self.robot.update(0.0)
        self.assertEqual(len(self.robot.driver.angles), 18)
        for deg in self.robot.driver.angles.values():
            self.assertGreater(deg, 30.0)
            self.assertLess(deg, 150.0)

    def test_walk_and_turn_stay_reachable(self):
        self.robot.command = GaitCommand(vx=80.0, vy=20.0, yaw_rate=0.3)
        self.robot.pose = BodyPose(z=10.0, roll=0.05, pitch=-0.05)
        for _ in range(200):
            self.robot.update(0.01)

    def test_feet_reach_their_targets(self):
        c = self.robot.config
        self.robot.command = GaitCommand(vx=60.0)
        self.robot.gait.phase = 0.3
        angles = self.robot.solve()
        targets = self.robot.foot_targets()
        for leg in c.legs:
            foot = leg_to_body_frame(
                leg_fk(angles[leg.name], c.coxa_length, c.femur_length, c.tibia_length), leg
            )
            for a, b in zip(foot, targets[leg.name]):
                self.assertAlmostEqual(a, b)

    def test_left_side_is_mirrored(self):
        c = self.robot.config
        self.robot.update(0.0)
        right = self.robot.driver.angles[(c.leg("RM").femur.board, c.leg("RM").femur.channel)]
        left = self.robot.driver.angles[(c.leg("LM").femur.board, c.leg("LM").femur.channel)]
        self.assertAlmostEqual(right - 90.0, 90.0 - left)

    def test_servo_mapping_clamps(self):
        joint = default_config().leg("RF").coxa
        self.assertEqual(ik_to_servo_deg(math.radians(200), joint), 180.0)
        self.assertEqual(ik_to_servo_deg(0.0, joint), 90.0)


if __name__ == "__main__":
    unittest.main()
