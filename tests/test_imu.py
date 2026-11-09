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
        self.assertEqual(c.handle("LEVEL 1"), "IMU present 1 level 1 feel 0 ready 1 axes +x+y+z zero 0 0")
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

    def test_bias_waits_until_the_robot_is_still(self):
        a = Attitude()
        flat = gravity_in_body(0.0, 0.0)
        for i in range(40):
            a.update(0.02, flat, (0.01, 0.0, 0.0))
        a.update(0.02, flat, (0.2, 0.0, 0.0))  # on bouge le robot
        for i in range(40):
            a.update(0.02, flat, (0.01, 0.0, 0.0))
        self.assertFalse(a.ready)  # la seconde de mesure a recommencé
        for i in range(15):
            a.update(0.02, flat, (0.01, 0.0, 0.0))
        self.assertTrue(a.ready)
        self.assertAlmostEqual(a._bias[0], 0.01)

    def test_imu_cal_measures_the_bias_again(self):
        c, imu = self.make()
        self.assertTrue(imu.attitude.ready)
        self.assertEqual(c.handle("IMU CAL"), "IMU present 1 level 0 feel 0 ready 0 axes +x+y+z zero 0 0")
        lines = run(c, 0.9)
        self.assertFalse(any(l.startswith("I ") for line in lines for l in line.split("\n")))
        run(c, 0.3)
        self.assertEqual(c.handle("IMU"), "IMU present 1 level 0 feel 0 ready 1 axes +x+y+z zero 0 0")
        self.assertTrue(c.handle("IMU FOO").startswith("ERR IMU"))

    def test_lost_gyro_is_not_trusted_until_it_answers_again(self):
        from hexapod.imu import FAIL_MAX, MPU6050

        class Bus:
            fail = False

            def writeto_mem(self, addr, reg, data):
                pass

            def readfrom_mem(self, addr, reg, n):
                if self.fail:
                    raise OSError("pas de réponse")
                return bytes([0, 0, 0, 0, 0x20, 0x00] + [0] * 8)  # à plat : 1 g vers le haut

        bus = Bus()
        imu = MPU6050(bus)
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", None, imu)
        c.handle("STAND")
        run(c, 4.0)
        c.handle("LEVEL 1")
        self.assertTrue(imu.attitude.ready)
        c.leveler.roll = 5 * DEG
        c.tilted = True
        bus.fail = True
        for _ in range(FAIL_MAX - 1):
            c.tick(0.02)
        self.assertTrue(imu.attitude.ready)  # quelques lectures perdues : rien ne change
        lines = run(c, 1.0)
        self.assertFalse(imu.attitude.ready)
        self.assertEqual(c.leveler.roll, 0.0)  # la correction est partie doucement
        self.assertFalse(c.tilted)
        self.assertTrue(any("TILT 0" in l for l in lines))
        self.assertFalse(any(l.startswith("I ") for line in lines[5:] for l in line.split("\n")))
        self.assertEqual(imu.errors, FAIL_MAX - 1 + 50)
        bus.fail = False
        c.tick(0.02)
        self.assertTrue(imu.attitude.ready)  # tout de suite : le biais est gardé

    def make_saved(self):
        import tempfile

        path = tempfile.mkdtemp() + "/calib.json"
        imu = SimIMU()
        c = Controller(default_config(), DryRunDriver(), path, None, imu)
        run(c, 1.2)
        return c, imu, path

    def test_axes_must_be_a_rotation(self):
        from hexapod.imu import format_axes, parse_axes

        self.assertEqual(parse_axes("+x+y+z"), ((0, 1), (1, 1), (2, 1)))
        self.assertEqual(parse_axes("y-xZ"), ((1, 1), (0, -1), (2, 1)))
        self.assertEqual(format_axes(parse_axes("-x-y+z")), "-x-y+z")
        n = 0
        for a in "xyz":
            for b in "xyz":
                for c in "xyz":
                    for signs in range(8):
                        spec = "".join("+-"[signs >> k & 1] + l for k, l in enumerate((a, b, c)))
                        try:
                            axes = parse_axes(spec)
                        except ValueError:
                            continue
                        n += 1
                        # Une rotation : le 3e axe est le produit vectoriel des deux premiers.
                        v = [[s if i == k else 0 for k in range(3)] for i, s in axes]
                        cross = [v[0][1] * v[1][2] - v[0][2] * v[1][1], v[0][2] * v[1][0] - v[0][0] * v[1][2],
                                 v[0][0] * v[1][1] - v[0][1] * v[1][0]]
                        self.assertEqual(cross, v[2], spec)
        self.assertEqual(n, 24)  # les 24 façons de visser la carte
        for bad in ("", "+x+y", "+x+y-z", "y x z", "+x+x+z", "abc", "+x+y+z+x", "x y z -"):
            self.assertRaises(ValueError, parse_axes, bad)

    def test_imu_axes_changes_the_reading_and_is_kept(self):
        c, imu, path = self.make_saved()
        c.handle("TILT 6 0")
        run(c, 3.0)
        self.assertAlmostEqual(imu.attitude.roll / DEG, 6, delta=0.5)
        self.assertTrue(c.handle("IMU AXES +x+y-z").startswith("ERR IMU : axes"))
        self.assertEqual(c.handle("IMU AXES -x -y +z"), "IMU present 1 level 0 feel 0 ready 0 axes -x-y+z zero 0 0")
        run(c, 3.0)
        self.assertAlmostEqual(imu.attitude.roll / DEG, -6, delta=0.5)  # carte tournée d'un demi-tour
        c.handle("TRIM RF femur 3")
        c.handle("SAVE")  # les trims enregistrés gardent les réglages du gyroscope
        imu2 = SimIMU()
        c2 = Controller(default_config(), DryRunDriver(), path, None, imu2)
        self.assertEqual(c2.handle("IMU"), "IMU present 1 level 0 feel 0 ready 0 axes -x-y+z zero 0 0")
        self.assertEqual(c2.robot.config.leg("RF").femur.trim_deg, 3)
        self.assertNotIn("imu", c2.handle("GETCAL"))  # GETCAL : seulement les servos

    def test_imu_zero_removes_the_flat_offset(self):
        c, imu, path = self.make_saved()
        c.handle("TILT 3 -2")  # ici : une carte vissée un peu de travers
        self.assertTrue(c.handle("IMU CAL").endswith("ready 0 axes +x+y+z zero 0 0"))
        self.assertTrue(c.handle("IMU ZERO").startswith("ERR IMU : gyroscope pas prêt"))
        run(c, 3.0)
        self.assertEqual(c.handle("IMU ZERO"), "IMU present 1 level 0 feel 0 ready 1 axes +x+y+z zero 30 -20")
        self.assertAlmostEqual(imu.attitude.roll, 0.0, delta=0.01 * DEG)
        self.assertAlmostEqual(imu.attitude.pitch, 0.0, delta=0.01 * DEG)
        # Debout avec une pose : le zéro ne compte pas la pose comme un écart.
        c.handle("STAND")
        c.handle("POSE 0 0 0 5 0 0")
        run(c, 5.0)
        self.assertAlmostEqual(imu.attitude.roll / DEG, 5, delta=0.5)
        self.assertTrue(c.handle("IMU ZERO").endswith("zero 30 -20"))
        c2 = Controller(default_config(), DryRunDriver(), path, None, SimIMU())
        self.assertTrue(c2.handle("IMU").endswith("zero 30 -20"))
        c.handle("TILT 14 0")
        run(c, 3.0)
        self.assertTrue(c.handle("IMU ZERO").startswith("ERR IMU : penché"))
        self.assertTrue(c.handle("IMU ZERO 0").endswith("zero 0 0"))
        c3 = Controller(default_config(), DryRunDriver(), path, None, SimIMU())
        self.assertTrue(c3.handle("IMU").endswith("zero 0 0"))

    def test_old_calibration_without_imu_settings_still_loads(self):
        import json
        import tempfile

        path = tempfile.mkdtemp() + "/calib.json"
        with open(path, "w") as f:
            f.write(json.dumps({"RF": [1, 2, 3], "invert": {"RF": [0, 1, 0]}}))
        c = Controller(default_config(), DryRunDriver(), path, None, SimIMU())
        self.assertEqual(c.handle("IMU"), "IMU present 1 level 0 feel 0 ready 0 axes +x+y+z zero 0 0")
        self.assertEqual(c.robot.config.leg("RF").femur.trim_deg, 2)
        with open(path, "w") as f:
            f.write(json.dumps({"RF": [1, 2, 3], "imu": {"axes": "+x+y-z", "zero": [1]}}))  # abîmés : ignorés
        c = Controller(default_config(), DryRunDriver(), path, None, SimIMU())
        self.assertTrue(c.handle("IMU").endswith("axes +x+y+z zero 0 0"))

    def test_level_off_releases_smoothly(self):
        c, imu = self.make()
        c.handle("TILT 6 0")
        c.handle("LEVEL 1")
        run(c, 4.0)
        self.assertAlmostEqual(c.leveler.roll / DEG, -6, delta=0.8)
        c.handle("LEVEL 0")
        before = c.leveler.roll
        c.tick(0.02)
        self.assertLess(abs(c.leveler.roll - before) / DEG, 0.3)  # pas d'à-coup
        run(c, 1.0)
        self.assertEqual(c.leveler.roll, 0.0)

    def test_bumpy_ground_is_levelled_while_walking(self):
        c, imu = self.make()
        self.assertEqual(c.handle("TERRAIN 1"), "TERRAIN 1")
        tilt = []
        for _ in range(60):
            c.handle("MOVE 100 0 0")
            tilt += [max(abs(v) for v in imu.slope) / DEG]
            run(c, 0.25)
        self.assertGreater(max(tilt), 4)  # la pente change sous le robot
        c.handle("LEVEL 1")
        body = []
        for _ in range(60):
            c.handle("MOVE 100 0 0")
            run(c, 0.25)
            body.append(max(abs(imu.attitude.roll), abs(imu.attitude.pitch)) / DEG)
        self.assertLess(sum(body[8:]) / len(body[8:]), 2.5)
        lines = run(c, 0.2)
        self.assertTrue(any(l.startswith("B ") for l in lines))

    def test_tilt_safety_stop(self):
        c, imu = self.make()
        imu.ground_roll = 15 * DEG
        c.handle("POSE 0 0 0 8 0 0")  # 15 + 8 = 23° > 20°
        lines = []
        for _ in range(8):
            c.handle("MOVE 80 0 0")
            lines += run(c, 0.25)
        self.assertIn("TILT 1", lines)
        self.assertTrue(c.robot.command.is_zero)
        imu.ground_roll = 0.0
        c.handle("POSE 0 0 0 0 0 0")
        lines = []
        for _ in range(12):
            c.handle("MOVE 80 0 0")
            lines += run(c, 0.25)
        self.assertIn("TILT 0", lines)
        self.assertFalse(c.robot.command.is_zero)

    def test_feeling_the_ground_helps_on_bumpy_ground(self):
        def walk(feel):
            c, imu = self.make()
            c.handle("TERRAIN 1")
            c.handle("LEVEL 1")
            self.assertEqual(c.handle("FEEL %d" % feel), "IMU present 1 level 1 feel %d ready 1 axes +x+y+z zero 0 0" % feel)
            tilt, air = [], []
            for i in range(750):
                if i % 25 == 0:
                    c.handle("MOVE 60 0 0")
                c.tick(0.02)
                tilt.append(max(abs(v) for v in (imu.attitude.roll, imu.attitude.pitch)))
                air.append(sum(1 for h in imu.air if h > 3))
            return sum(tilt) / len(tilt), sum(air) / len(air), c.robot
        tilt0, air0, _ = walk(0)
        tilt1, air1, robot = walk(1)
        self.assertLess(tilt1, tilt0)
        self.assertLess(air1, air0)
        self.assertTrue(any(robot.ground.values()))  # il a bien deviné des hauteurs

    def test_feeling_does_not_slow_walk_with_uneven_ticks(self):
        # Appli de démo : ticks de 20 ms suivis de petits bouts (page lente).
        def speed(feel):
            c, imu = self.make()
            c.handle("TERRAIN 1")
            c.handle("LEVEL 1")
            c.handle("FEEL %d" % feel)
            t = d = 0.0
            for i in range(400):
                if i % 20 == 0:
                    c.handle("MOVE 100 0 0")
                for dt in (0.02, 0.002 + 0.008 * (i % 3)):
                    c.tick(dt)
                    t += dt
                    d += c.robot.travel[0]
            return d / t
        self.assertGreater(speed(1), 0.95 * speed(0))

    def test_real_driver_runs_in_the_control_loop(self):
        import struct
        from hexapod.imu import MPU6050

        class FakeI2C:
            def writeto_mem(self, addr, reg, data):
                pass

            def scan(self):
                return [0x68]

            def readfrom_mem(self, addr, reg, n):
                return struct.pack(">hhhhhhh", 0, 0, 8192, 0, 0, 0, 0)  # à plat, immobile

        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", None, MPU6050(FakeI2C()))
        c.handle("STAND")
        lines = run(c, 2.0)
        self.assertTrue(any(l.startswith("I ") for l in lines))
        self.assertTrue(any(l.startswith("M ") for l in lines))

    def test_yaw_while_tilted_does_not_fool_the_attitude(self):
        from hexapod.imu import euler_rates_to_body
        a = Attitude()
        roll, pitch, yaw = 0.0, 10 * DEG, 0.0
        for _ in range(60):
            a.update(0.02, gravity_in_body(roll, pitch), (0.0, 0.0, 0.0))
        for _ in range(500):  # tourne à 0,6 rad/s sur une pente de 10°
            a.update(0.02, gravity_in_body(roll, pitch), euler_rates_to_body(roll, pitch, 0.0, 0.0, 0.6))
        self.assertAlmostEqual(a.roll / DEG, 0, delta=0.3)
        self.assertAlmostEqual(a.pitch / DEG, 10, delta=0.3)

    def test_absent(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json")
        self.assertEqual(c.handle("IMU"), "IMU present 0 level 0 feel 0 ready 0 axes +x+y+z zero 0 0")
        self.assertTrue(c.handle("LEVEL 1").startswith("ERR"))


if __name__ == "__main__":
    unittest.main()
