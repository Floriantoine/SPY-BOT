import math
import unittest

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.sensor import SimRanger
from hexapod.servo import DryRunDriver


def run(ctrl, seconds, dt=0.02):
    lines = []
    for _ in range(int(round(seconds / dt))):
        out = ctrl.tick(dt)
        if out:
            lines += out.split("\n")
    return lines


def make(ranger):
    c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", ranger)
    c.handle("STAND")
    run(c, 4.0)
    return c


class GotoTest(unittest.TestCase):
    def test_reaches_goal_without_obstacles(self):
        c = make(None)
        self.assertEqual(c.handle("GOTO 400 200"), "NAV goto 400 200")
        lines = run(c, 25.0)
        self.assertIn("NAV arrived", lines)
        x, y, _ = c.odom
        self.assertLess(math.hypot(x - 400, y - 200), 40)

    def test_goes_around_obstacle(self):
        c = make(SimRanger([(450, 0, 60)]))
        c.handle("GOTO 900 0")
        self.assertTrue(c.scanner.enabled)  # allumé tout seul
        closest = 1e9
        for _ in range(60 * 50 // 10):
            run(c, 0.2)
            x, y, _ = c.odom
            closest = min(closest, math.hypot(x - 450, y) - 60)
            if not c.nav.active:
                break
        self.assertFalse(c.nav.active, "but non atteint")
        self.assertGreater(closest, 150)  # les pieds (156 mm) n'ont pas touché

    def test_move_takes_back_control(self):
        c = make(None)
        c.handle("GOTO 400 0")
        run(c, 1.0)
        c.handle("MOVE 0 0 0")
        self.assertFalse(c.nav.active)

    def test_refused_when_sitting(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        self.assertTrue(c.handle("GOTO 100 0").startswith("ERR"))


    def test_gives_up_on_goal_inside_obstacle(self):
        c = make(SimRanger([(400, 0, 80)]))
        c.handle("GOTO 400 0")
        lines = run(c, 40.0)
        self.assertIn("NAV stuck", lines)
        self.assertFalse(c.nav.active)


class FollowTest(unittest.TestCase):
    def test_keeps_following_person_not_nearer_obstacle(self):
        r = SimRanger([(500, -300, 50)])
        c = make(r)
        c.handle("PERSON 600 150")
        c.handle("FOLLOW")
        run(c, 6.0)
        c.handle("PERSON 700 700")  # passe à côté de l'obstacle, plus près du robot
        run(c, 20.0)
        x, y, th = c.odom
        bearing = math.atan2(700 - y, 700 - x) - th
        self.assertLess(abs(math.atan2(math.sin(bearing), math.cos(bearing))), 0.5)

    def test_needs_sensor(self):
        self.assertTrue(make(None).handle("FOLLOW").startswith("ERR"))

    def test_follows_walking_person(self):
        r = SimRanger()
        c = make(r)
        c.handle("PERSON 600 0")
        c.handle("FOLLOW")
        run(c, 6.0)
        x, y, _ = c.odom
        self.assertLess(abs(math.hypot(600 - x, -y) - 70 - 400), 60)  # à ~40 cm de ses jambes
        c.handle("PERSON 1200 500")  # la personne s'éloigne en tournant
        run(c, 25.0)
        x, y, th = c.odom
        px, py = r.person
        self.assertLess(math.hypot(px - x, py - y) - 70, 520)
        bearing = math.atan2(py - y, px - x) - th
        self.assertLess(abs(math.atan2(math.sin(bearing), math.cos(bearing))), 0.4)


if __name__ == "__main__":
    unittest.main()
