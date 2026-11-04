import math
import unittest

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.imu import Attitude, SimIMU, gravity_in_body, _s16
from hexapod.servo import DryRunDriver

DEG = math.pi / 180


def run(ctrl, seconds, dt=0.02):
    lines = []
    for _ in range(int(round(seconds / dt))):
        out = ctrl.tick(dt)
        if out:
            lines += out.split("\n")
    return lines


class AttitudeTest(unittest.TestCase):
    def test_reads_tilt_from_gravity(self):
        a = Attitude()
        g = gravity_in_body(5 * DEG, -3 * DEG)
        for _ in range(60):
            a.update(0.02, g, (0.01, -0.02, 0.0))  # gyroscope avec un biais
        self.assertTrue(a.ready)
        for _ in range(200):
            a.update(0.02, g, (0.01, -0.02, 0.0))
        self.assertAlmostEqual(a.roll / DEG, 5, delta=0.3)
        self.assertAlmostEqual(a.pitch / DEG, -3, delta=0.3)

    def test_signed_registers(self):
        self.assertEqual(_s16(0xFF, 0xFF), -1)
        self.assertEqual(_s16(0x20, 0x00), 8192)


class LevelTest(unittest.TestCase):
    def make(self):
        imu = SimIMU()
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", None, imu)
        c.handle("STAND")
        run(c, 4.0)
        return c, imu

    def test_pose_roll_reads_on_imu(self):
        c, imu = self.make()
        c.handle("POSE 0 0 0 5 0 0")
        run(c, 3.0)
        self.assertAlmostEqual(imu.attitude.roll / DEG, 5, delta=0.5)

    def test_levels_body_on_slope(self):
        c, imu = self.make()
        self.assertEqual(c.handle("TILT 6 -4"), "OK TILT 6 -4")
        run(c, 2.0)
        self.assertAlmostEqual(imu.attitude.roll / DEG, 6, delta=1)
        self.assertEqual(c.handle("LEVEL 1"), "IMU present 1 level 1")
        lines = run(c, 4.0)
        self.assertLess(abs(imu.attitude.roll / DEG), 0.7)
        self.assertLess(abs(imu.attitude.pitch / DEG), 0.7)
        self.assertAlmostEqual(c.robot.pose.roll / DEG, -6, delta=0.8)
        self.assertTrue(any(l.startswith("I ") for l in lines))
        # Et ça marche toujours en marchant.
        for _ in range(20):
            c.handle("MOVE 60 0 0")
            run(c, 0.1)
        self.assertLess(abs(imu.attitude.roll / DEG), 1)

    def test_absent(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        self.assertEqual(c.handle("IMU"), "IMU present 0 level 0")
        self.assertTrue(c.handle("LEVEL 1").startswith("ERR"))


if __name__ == "__main__":
    unittest.main()
