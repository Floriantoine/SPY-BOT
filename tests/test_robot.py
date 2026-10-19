import math
import unittest

from hexapod.servo import PCA9685Driver
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

    def test_standby_pose_matches_reference_robot(self):
        # Posture debout de hexapod-v2-7697 : fémur relevé de 30°, tibia
        # incliné de 15° vers l'extérieur, soit coxa 0°, fémur +15° et
        # tibia -15° par rapport au milieu des servos.
        self.robot.update(0.0)
        for leg in self.robot.config.legs:
            servos = self.robot.driver.angles
            self.assertAlmostEqual(servos[(leg.coxa.board, leg.coxa.channel)], 90.0)
            self.assertAlmostEqual(servos[(leg.femur.board, leg.femur.channel)], 75.0)
            self.assertAlmostEqual(servos[(leg.tibia.board, leg.tibia.channel)], 75.0)

    def test_eighteen_distinct_channels(self):
        c = self.robot.config
        channels = {(j.board, j.channel) for leg in c.legs for j in (leg.coxa, leg.femur, leg.tibia)}
        self.assertEqual(len(channels), 18)

    def test_scaled_dimensions_stay_reachable(self):
        robot = Hexapod(default_config({"femur": 85.2, "tibia": 178.1, "coxa": 56.0}))
        robot.command = GaitCommand(vx=120.0, yaw_rate=0.5)
        for _ in range(100):
            robot.update(0.02)

    def test_servo_mapping_clamps(self):
        joint = default_config().leg("RF").coxa
        self.assertEqual(ik_to_servo_deg(math.radians(200), joint), 135.0)
        self.assertEqual(ik_to_servo_deg(0.0, joint), 90.0)


if __name__ == "__main__":
    unittest.main()


class PostureLimitsTest(unittest.TestCase):
    def test_sit_and_stand_stay_inside_servo_limits(self):
        # Un servo en butée déformerait la posture : assis comme debout, chaque
        # angle demandé doit rester dans la course du servo, avec 3° de marge.
        config = default_config()
        robot = Hexapod(config)
        for height in (0.0, -config.sit_drop):
            robot.pose = BodyPose(0, 0, height, 0, 0, 0)
            angles = robot.solve()
            for leg in config.legs:
                a = angles[leg.name]
                for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                    raw = 90.0 + joint.direction * (math.degrees(value) - joint.zero_deg)
                    self.assertGreater(raw, joint.min_deg + 3, (height, leg.name))
                    self.assertLess(raw, joint.max_deg - 3, (height, leg.name))


class MissingBoardTest(unittest.TestCase):
    """Une PCA9685 absente du bus ne doit pas empêcher le robot de démarrer."""

    def test_missing_board_is_skipped(self):
        class FakeI2C:
            def __init__(self):
                self.writes = []

            def writeto_mem(self, address, reg, data):
                if address != 0x40:
                    raise OSError(19)
                self.writes.append((address, reg))

            def readfrom_mem(self, address, reg, n):
                return bytes(n)

        import time
        if not hasattr(time, "sleep_us"):
            time.sleep_us = lambda us: None
        i2c = FakeI2C()
        driver = PCA9685Driver(i2c)
        self.assertEqual(driver.missing, [0x41])
        driver.set_angle(1, 5, 90.0)  # carte absente : ignoré
        driver.set_angle(0, 5, 90.0)
        driver.relax()
        self.assertTrue(all(a == 0x40 for a, _ in i2c.writes))
