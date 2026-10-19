import json
import os
import tempfile
import unittest

from hexapod import default_config
from hexapod.controller import CALIBRATION, OFF, SIT, STAND, WAKE, Controller
from hexapod.servo import DryRunDriver


class RelaxDriver(DryRunDriver):
    relaxed = False

    def relax(self):
        self.relaxed = True


def run(ctrl, seconds, dt=0.02):
    lines = []
    for _ in range(int(seconds / dt)):
        line = ctrl.tick(dt)
        if line:
            lines.append(line)
    return lines


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.calib = os.path.join(self.tmp, "calib.json")
        self.driver = RelaxDriver()
        self.ctrl = Controller(default_config(), self.driver, self.calib)

    def test_starts_relaxed_and_sends_nothing_to_servos(self):
        self.assertEqual(self.ctrl.mode, OFF)
        run(self.ctrl, 0.5)
        self.assertEqual(self.driver.angles, {})

    def test_stand_from_off_powers_on_first(self):
        c = self.ctrl
        self.assertEqual(c.handle("STAND"), "OK STAND")
        self.assertEqual(c.mode, WAKE)
        run(c, 4.0)
        self.assertEqual(c.mode, STAND)

    def test_power_on_enables_legs_one_after_another(self):
        c = self.ctrl
        self.assertEqual(c.handle("POWER 1"), "OK POWER 1")
        self.assertEqual(c.mode, WAKE)
        self.assertEqual(len(self.driver.angles), 3)  # une seule patte active
        run(c, 0.6)
        self.assertEqual(len(self.driver.angles), 9)
        run(c, 1.0)
        self.assertEqual(len(self.driver.angles), 18)
        self.assertEqual(c.mode, SIT)

    def test_power_on_from_calibration_moves_smoothly(self):
        c = self.ctrl
        c.handle("CAL")
        leg = c.robot.config.leg("RF")
        key = (leg.femur.board, leg.femur.channel)
        previous = self.driver.angles[key]
        c.handle("POWER 1")
        steps = []
        for _ in range(60):
            c.tick(0.02)
            steps.append(abs(self.driver.angles[key] - previous))
            previous = self.driver.angles[key]
        self.assertEqual(c.mode, SIT)
        self.assertLess(max(steps), 3.0)  # pas plus de 3° par pas de 20 ms

    def test_power_stand_walk_sit_power_off(self):
        c = self.ctrl
        self.assertEqual(c.handle("POWER 1"), "OK POWER 1")
        run(c, 1.5)
        self.assertEqual(c.mode, SIT)
        self.assertEqual(len(self.driver.angles), 18)
        c.handle("STAND")
        run(c, 3.0)
        self.assertEqual(c.mode, STAND)
        c.handle("MOVE 60 0 0")
        run(c, 0.2)
        self.assertNotEqual(c.robot.gait.phase, 0.0)
        c.handle("SIT")
        run(c, 4.0)
        self.assertEqual(c.mode, SIT)
        self.assertEqual(c.handle("POWER 0"), "OK POWER 0")
        self.assertTrue(self.driver.relaxed)
        self.assertEqual(c.mode, OFF)

    def test_turn_in_place(self):
        c = self.ctrl
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 4.0)
        for _ in range(20):
            c.handle("MOVE 0 0 30")
            run(c, 0.1)
        self.assertAlmostEqual(c.robot.command.yaw_rate, 0.5236, places=3)
        self.assertEqual(c.robot.command.vx, 0.0)

    def test_watchdog_stops_robot_when_link_is_lost(self):
        c = self.ctrl
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 4.0)
        c.handle("MOVE 60 0 0")
        run(c, 2.0)
        self.assertTrue(c.robot.command.is_zero)
        self.assertIn(c.robot.gait.phase, (0.0, 0.5))

    def test_move_refused_when_not_standing(self):
        self.assertTrue(self.ctrl.handle("MOVE 10 0 0").startswith("ERR"))

    def test_telemetry_format(self):
        line = run(self.ctrl, 0.1)[0].split()
        self.assertEqual(line[0], "T")
        self.assertEqual(line[1], OFF)
        self.assertEqual(len(line), 3 + 18)

    def test_calibration_pose_is_easy_to_check(self):
        # Fémur à plat et tibia vertical : le servo de fémur (milieu = fémur
        # relevé de 15°, monté à l'envers) est à 105°, les autres au milieu.
        self.ctrl.handle("CAL")
        for leg in self.ctrl.robot.config.legs:
            angle = lambda j: self.driver.angles[(j.board, j.channel)]
            self.assertAlmostEqual(angle(leg.coxa), 90.0)
            self.assertAlmostEqual(angle(leg.femur), 105.0)
            self.assertAlmostEqual(angle(leg.tibia), 90.0)

    def test_calibration_round_trip(self):
        c = self.ctrl
        self.assertEqual(c.handle("CAL"), "OK CAL")
        self.assertEqual(c.mode, CALIBRATION)
        leg = c.robot.config.leg("LM")
        before = self.driver.angles[(leg.femur.board, leg.femur.channel)]
        self.assertAlmostEqual(before, 105.0)
        self.assertEqual(c.handle("TRIM lm femur -4.5"), "OK TRIM LM femur -4.5")
        self.assertAlmostEqual(self.driver.angles[(leg.femur.board, leg.femur.channel)], 100.5)
        c.handle("SAVE")
        with open(self.calib) as f:
            self.assertEqual(json.load(f)["LM"], [0.0, -4.5, 0.0])

        other = Controller(default_config(), DryRunDriver(), self.calib)
        self.assertEqual(other.robot.config.leg("LM").femur.trim_deg, -4.5)
        self.assertEqual(json.loads(other.handle("GETCAL")[4:])["LM"], [0.0, -4.5, 0.0])

    def test_servo_test_moves_only_that_servo(self):
        c = self.ctrl
        self.assertEqual(c.handle("TEST rr tibia"), "OK TEST RR tibia")
        leg = c.robot.config.leg("RR")
        target = (leg.tibia.board, leg.tibia.channel)
        seen = {}
        for _ in range(int(1.0 / 0.02)):
            c.tick(0.02)
            for key, deg in self.driver.angles.items():
                seen.setdefault(key, set()).add(round(deg, 3))
        self.assertGreater(max(seen[target]) - min(seen[target]), 30.0)
        self.assertTrue(all(len(v) == 1 for k, v in seen.items() if k != target))
        run(c, 3.0)
        self.assertEqual(c.mode, CALIBRATION)

    def test_servo_test_refused_while_standing(self):
        c = self.ctrl
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 4.0)
        self.assertTrue(c.handle("TEST RF coxa").startswith("ERR"))

    def test_bad_commands(self):
        c = self.ctrl
        self.assertTrue(c.handle("FLY").startswith("ERR"))
        self.assertTrue(c.handle("TRIM XX femur 1").startswith("ERR"))
        self.assertTrue(c.handle("TRIM RF knee 1").startswith("ERR"))
        self.assertTrue(c.handle("MOVE 1").startswith("ERR"))
        self.assertIsNone(c.handle("   "))

    def test_geometry(self):
        geom = json.loads(self.ctrl.handle("GEOM")[5:])
        self.assertEqual(len(geom["legs"]), 6)
        self.assertAlmostEqual(geom["tibia"], 89.07)
        self.assertEqual(geom["addresses"], [0x40, 0x41])
        rf = next(leg for leg in geom["legs"] if leg["name"] == "RF")
        self.assertEqual(rf["servos"], [[0, 5], [0, 6], [0, 7]])


if __name__ == "__main__":
    unittest.main()
