import math
import unittest

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.imu import SimIMU
from hexapod.servo import DryRunDriver
from hexapod import terrain


class TerrainTest(unittest.TestCase):
    def test_robot_rests_on_its_feet(self):
        feet = [(120, 80, -50), (-120, 80, -50), (0, -140, -50), (120, -80, -50), (-120, -80, -50), (0, 140, -50)]
        for odom in ((600, 300, 0.3), (-900, 1200, 2.0), (0, 0, 0)):
            a, b, c, roll, pitch, air = terrain.rest_on(feet, odom)
            self.assertAlmostEqual(min(air), 0.0, places=6)  # au moins un pied touche
            self.assertGreaterEqual(min(air), -1e-6)  # aucun ne s'enfonce
            self.assertLess(max(abs(roll), abs(pitch)), math.radians(20))
        # Trois pieds : le plan passe exactement par le terrain sous chacun.
        _, _, _, _, _, air = terrain.rest_on(feet[:3], (600, 300, 0.3))
        for v in air:
            self.assertAlmostEqual(v, 0.0, places=6)

    def test_flat_start(self):
        self.assertLess(abs(terrain.height(0, 0)), 40)

    def test_controller_sends_map_and_plane(self):
        c = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-test-calib.json", None, SimIMU())
        c.handle("POWER 1")
        c.handle("STAND")
        for _ in range(150):
            c.tick(0.02)
        self.assertEqual(c.handle("TERRAIN 1"), "TERRAIN 1")
        lines = []
        for i in range(200):
            if i % 25 == 0:
                c.handle("MOVE 100 0 0")
            out = c.tick(0.02)
            if out:
                lines += out.split("\n")
        self.assertTrue(lines[0].startswith("TERRAINMAP 3 "))
        self.assertTrue(any(l.startswith("G ") for l in lines))
        self.assertGreater(max(abs(v) for v in c.imu.slope), math.radians(0.5))


if __name__ == "__main__":
    unittest.main()
