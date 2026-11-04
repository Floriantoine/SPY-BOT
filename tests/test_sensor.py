import math
import unittest

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.gait import GaitCommand
from hexapod.sensor import CHIP_X, PAN_LIMIT, PAN_SERVO, MIN_RANGE, Scanner, SimRanger
from hexapod.servo import DryRunDriver


def run(ctrl, seconds, move=None, dt=0.02):
    lines = []
    for i in range(int(round(seconds / dt))):
        if move and i % 5 == 0:
            ctrl.handle("MOVE %g %g %g" % move)
        out = ctrl.tick(dt)
        if out:
            lines += out.split("\n")
    return lines


class SimRangerTest(unittest.TestCase):
    def test_distance_to_cylinder_ahead(self):
        r = SimRanger([(500, 0, 50)])
        self.assertAlmostEqual(r.read(0, 0, 0), 450, delta=1)

    def test_nothing_in_view(self):
        r = SimRanger([(500, 0, 50)])
        self.assertGreater(r.read(0, 0, math.pi), 4000)

    def test_wide_beam_sees_slightly_off_axis(self):
        r = SimRanger([(500, 60, 20)])
        self.assertLess(r.read(0, 0, 0), 600)


class ScannerTest(unittest.TestCase):
    def test_sweeps_within_servo_range(self):
        drv = DryRunDriver()
        s = Scanner(SimRanger(), drv)
        s.enable(True)
        seen = set()
        for _ in range(200):
            s.tick(0.05, (0, 0, 0), GaitCommand())
            seen.add(s.angle)
        self.assertEqual(min(seen), -PAN_LIMIT)
        self.assertEqual(max(seen), PAN_LIMIT)
        self.assertIn(PAN_SERVO, drv.angles)

    def test_ignores_own_legs(self):
        s = Scanner(SimRanger([(CHIP_X + MIN_RANGE - 30, 0, 20)]))
        s.enable(True)
        for _ in range(100):
            s.tick(0.05, (0, 0, 0), GaitCommand())
        self.assertEqual(s.points, [])

    def test_blocked_only_toward_obstacle(self):
        s = Scanner(SimRanger([(220, 0, 30)]))
        s.enable(True)
        for _ in range(60):
            s.tick(0.05, (0, 0, 0), GaitCommand())
        self.assertTrue(s.check((0, 0, 0), GaitCommand(80, 0, 0)))
        self.assertFalse(s.check((0, 0, 0), GaitCommand(-80, 0, 0)))
        self.assertFalse(s.check((0, 0, 0), GaitCommand(0, 0, 0.5)))
        s.stop_dist = 100
        self.assertFalse(s.check((0, 0, 0), GaitCommand(80, 0, 0)))


class ControllerSensorTest(unittest.TestCase):
    def make(self, ranger):
        return Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", ranger)

    def test_absent_sensor(self):
        c = self.make(None)
        self.assertEqual(c.handle("SENSOR"), "SENSOR present 0 on 0 stop 200")
        self.assertTrue(c.handle("SENSOR 1").startswith("ERR"))
        self.assertTrue(c.handle("OBST").startswith("ERR"))

    def test_obstacle_commands(self):
        c = self.make(SimRanger())
        self.assertEqual(c.handle("OBST ADD 300 -50 40"), "OBST 300 -50 40")
        self.assertEqual(c.handle("OBST CLEAR"), "OBST ")
        self.assertEqual(c.handle("SENSOR STOP 5000"), "SENSOR present 1 on 0 stop 600")

    def test_stops_in_front_of_obstacle_but_can_turn(self):
        c = self.make(SimRanger([(500, 0, 40)]))
        c.handle("SENSOR 1")
        c.handle("STAND")
        run(c, 4.0)
        lines = run(c, 8.0, (80, 0, 0))
        self.assertIn("OBSTACLE 1", lines)
        self.assertTrue(any(l.startswith("S ") for l in lines))
        self.assertTrue(any(l.startswith("P ") for l in lines))
        x = c.odom[0]
        front = 500 - 40 - CHIP_X
        self.assertLess(x, front - 120)  # arrêté avant de toucher
        self.assertGreater(x, front - 320)  # mais assez près
        run(c, 3.0, (80, 0, 0))
        self.assertAlmostEqual(c.odom[0], x, delta=5)  # reste arrêté
        th = c.odom[2]
        run(c, 3.0, (0, 0, 30))
        self.assertGreater(c.odom[2], th + 0.5)  # peut tourner

    def test_disabled_sensor_does_not_stop(self):
        c = self.make(SimRanger([(400, 0, 40)]))
        c.handle("STAND")
        run(c, 4.0)
        lines = run(c, 6.0, (80, 0, 0))
        self.assertNotIn("OBSTACLE 1", lines)
        self.assertGreater(c.odom[0], 350)

    def test_odometry_follows_walk(self):
        c = self.make(None)
        c.handle("STAND")
        run(c, 4.0)
        run(c, 5.0, (60, 0, 0))
        self.assertGreater(c.odom[0], 200)
        self.assertLess(abs(c.odom[1]), 5)


if __name__ == "__main__":
    unittest.main()
