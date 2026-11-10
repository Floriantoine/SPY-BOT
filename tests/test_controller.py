import json
import math
import os
import tempfile
import unittest

from hexapod import default_config
from hexapod.controller import CALIBRATION, OFF, SIT, STAND, WAKE, Controller
from hexapod.servo import DryRunDriver, ik_to_servo_deg


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

    def test_servos_resent_after_power_cycle(self):
        # Immobile, les angles ne sont plus renvoyés ; après un relâchement,
        # ils doivent l'être de nouveau.
        self.ctrl.handle("STAND")
        run(self.ctrl, 4.0)
        self.driver.angles.clear()
        run(self.ctrl, 0.2)
        self.assertEqual(self.driver.angles, {})  # rien de nouveau à envoyer
        self.ctrl.handle("POWER 0")
        self.ctrl.handle("STAND")
        run(self.ctrl, 4.0)
        self.assertEqual(len(self.driver.angles), 18)

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
        run(c, 4.5)  # 1 s sans MOVE, puis un pas par patte pour revenir au neutre
        self.assertTrue(c.robot.command.is_zero)
        self.assertTrue(c.robot.at_rest)

    def test_move_refused_when_not_standing(self):
        self.assertTrue(self.ctrl.handle("MOVE 10 0 0").startswith("ERR"))

    def test_move_refuses_nan_and_inf(self):
        c = self.ctrl
        c.handle("STAND")
        run(c, 4.0)
        for line in ("MOVE nan 0 0", "MOVE 0 inf 0", "MOVE 0 0 -inf", "MOVE NaN nan nan"):
            self.assertEqual(c.handle(line), "ERR MOVE : vitesse invalide")
        run(c, 0.5)  # aucun tick ne lève d'erreur
        c.handle("MOVE 60 0 0")
        run(c, 0.5)
        self.assertFalse(c.robot.command.is_zero)  # le robot marche encore

    def test_telemetry_format(self):
        line = run(self.ctrl, 0.1)[0].split("\n")[0].split()
        self.assertEqual(line[0], "T")
        self.assertEqual(line[1], OFF)
        self.assertEqual(len(line), 3 + 18)

    def test_calibration_pose_is_easy_to_check(self):
        # Fémur à plat et tibia vertical : tous les servos au milieu.
        self.ctrl.handle("CAL")
        for leg in self.ctrl.robot.config.legs:
            angle = lambda j: self.driver.angles[(j.board, j.channel)]
            self.assertAlmostEqual(angle(leg.coxa), 90.0)
            self.assertAlmostEqual(angle(leg.femur), 90.0)
            self.assertAlmostEqual(angle(leg.tibia), 90.0)

    def test_calibration_round_trip(self):
        c = self.ctrl
        self.assertEqual(c.handle("CAL"), "OK CAL")
        self.assertEqual(c.mode, CALIBRATION)
        leg = c.robot.config.leg("LM")
        before = self.driver.angles[(leg.femur.board, leg.femur.channel)]
        self.assertAlmostEqual(before, 90.0)
        self.assertEqual(c.handle("TRIM lm femur -4.5"), "OK TRIM LM femur -4.5")
        for _ in range(20):  # le servo rejoint sa consigne en douceur (SLEW)
            c.tick(0.02)
        self.assertAlmostEqual(self.driver.angles[(leg.femur.board, leg.femur.channel)], 85.5)
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
        self.assertAlmostEqual(geom["tibia"], 94.5)
        self.assertEqual(geom["addresses"], [0x40, 0x41])
        rf = next(leg for leg in geom["legs"] if leg["name"] == "RF")
        self.assertEqual(rf["servos"], [[0, 5], [0, 6], [0, 7]])


if __name__ == "__main__":
    unittest.main()


class InvertTest(unittest.TestCase):
    def test_invert_flips_servo_direction_and_is_saved(self):
        import os
        import tempfile

        path = os.path.join(tempfile.mkdtemp(), "calib.json")
        driver = DryRunDriver()
        ctrl = Controller(default_config(), driver, path)
        ctrl.handle("CAL")
        ctrl.handle("TRIM RF femur 15")
        for _ in range(20):  # le servo rejoint sa consigne en douceur (SLEW)
            ctrl.tick(0.02)
        leg = ctrl.robot.config.leg("RF")
        key = (leg.femur.board, leg.femur.channel)
        self.assertAlmostEqual(driver.angles[key], 105.0, places=3)
        self.assertEqual(ctrl.handle("INVERT RF femur 1"), "OK INVERT RF femur 1")
        self.assertTrue(leg.femur.inverted)
        # Sens opposé : un fémur relevé de 10° passe de 80° à 100° (+ trim).
        self.assertAlmostEqual(ik_to_servo_deg(math.radians(10), leg.femur), 115.0, places=3)
        self.assertEqual(ctrl.calibration()["invert"]["RF"], [0, 1, 0])
        ctrl.handle("SAVE")
        again = Controller(default_config(), DryRunDriver(), path)
        self.assertTrue(again.robot.config.leg("RF").femur.inverted)
        self.assertFalse(again.robot.config.leg("RM").femur.inverted)


class LegsCommandTest(unittest.TestCase):
    def test_walk_on_four_legs(self):
        c = Controller(default_config(), DryRunDriver(), os.path.join(tempfile.mkdtemp(), "c.json"))
        c.handle("STAND")
        for _ in range(250):
            c.tick(0.02)
        self.assertEqual(c.handle("LEGS 4"), "OK LEGS 4")
        for _ in range(100):
            c.handle("MOVE 100 0 0")
            c.tick(0.02)
        self.assertNotEqual(c.robot.gait.phase, 0.0)
        self.assertTrue(c.handle("LEGS 6").startswith("ERR"))  # en marche
        c.handle("SIT")
        for _ in range(400):
            c.tick(0.02)
        self.assertEqual(c.mode, SIT)
        self.assertEqual(c.handle("LEGS 6"), "OK LEGS 6")
        self.assertTrue(c.handle("LEGS 5").startswith("ERR"))


class SmoothStartStopTest(unittest.TestCase):
    def test_no_servo_jump_when_starting_or_stopping(self):
        # Démarrer ou s'arrêter d'un coup faisait sauter les pieds d'un
        # demi-pas : la consigne suit maintenant la demande en douceur.
        for legs in ("6", "4"):
            driver = DryRunDriver()
            c = Controller(default_config(), driver, os.path.join(tempfile.mkdtemp(), "c.json"))
            c.handle("STAND")
            for _ in range(250):
                c.tick(0.02)
            c.handle("LEGS " + legs)
            for _ in range(60):
                c.tick(0.02)
            prev = dict(driver.angles)
            for i in range(600):
                c.handle("MOVE 150 0 0" if i < 300 else "MOVE 0 0 0")
                c.tick(0.02)
                for key, value in driver.angles.items():
                    self.assertLess(abs(value - prev[key]), 8.0, (legs, i, key))
                prev = dict(driver.angles)
            self.assertTrue(c.robot.gait.at_rest)


class StopTest(unittest.TestCase):
    def test_stops_within_one_step_after_release(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 3.0)
        for release in (0.2, 0.5, 0.65):
            for i in range(int((2.0 + release) / 0.02)):
                if i % 25 == 0:
                    c.handle("MOVE 100 0 0")
                c.tick(0.02)
            c.handle("MOVE 0 0 0")
            t = 0.0
            while not c.robot.at_rest:
                c.tick(0.02)
                t += 0.02
                self.assertLess(t, 0.9, release)  # le pas en cours, plus au plus un

    def test_no_body_jolt_when_restarting(self):
        # Repartir après un arrêt : le corps ne se fige jamais un tick au
        # changement de série de pattes (à-coup avant/arrière).
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 3.0)
        speeds = []
        for line, seconds in (("MOVE 100 0 0", 2.0), ("MOVE 0 0 0", 1.5), ("MOVE 100 0 0", 1.5), ("MOVE -100 0 0", 2.0)):
            for i in range(int(seconds / 0.02)):
                if i % 25 == 0:
                    c.handle(line)
                c.tick(0.02)
                speeds.append(c.robot.travel[0] / 0.02)
        jumps = [abs(b - a) / 0.02 for a, b in zip(speeds, speeds[1:])]
        self.assertLessEqual(max(jumps), 650)  # mm/s² : la décélération voulue, pas plus

    def test_measures_line(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        c.handle("POWER 1")
        c.handle("STAND")
        run(c, 3.0)
        lines = []
        for i in range(100):
            if i % 25 == 0:
                c.handle("MOVE 60 0 0")
            out = c.tick(0.02)
            lines += [l for l in (out or "").split("\n") if l.startswith("M ")]
        speed, wanted, margin, pose, lift, tick = map(int, lines[-1].split()[1:])
        self.assertEqual(wanted, 60)
        self.assertAlmostEqual(speed, 60, delta=3)
        self.assertGreater(margin, 0)
        self.assertEqual((pose, lift), (100, 100))
        self.assertGreater(tick, 0)


class ProfTest(unittest.TestCase):
    def test_prof_lines(self):
        from hexapod import default_config
        from hexapod.controller import Controller

        class Driver:
            errors = 0

            def set_angle(self, *a):
                pass

            def relax(self):
                pass

        import tempfile, os
        c = Controller(default_config(), Driver(), os.path.join(tempfile.mkdtemp(), "calib.json"))
        self.assertEqual(c.handle("PROF 1"), "OK PROF 1")
        c.handle("STAND")
        out = "\n".join(filter(None, (c.tick(0.02) for _ in range(200))))
        prof = [l for l in out.split("\n") if l.startswith("PROF")]
        self.assertTrue(prof)
        self.assertIn(" ik ", prof[-1])
        self.assertEqual(c.handle("PROF 0"), "OK PROF 0")
        self.assertNotIn("timed", repr(c.robot.solve_limited))
