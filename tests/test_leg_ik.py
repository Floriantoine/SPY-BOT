import math
import unittest

from hexapod.leg_ik import LegAngles, UnreachableError, leg_fk, leg_ik

L = (50.0, 80.0, 120.0)


class LegIKTest(unittest.TestCase):
    def assertPointAlmostEqual(self, a, b, places=6):
        for u, v in zip(a, b):
            self.assertAlmostEqual(u, v, places=places)

    def test_ik_then_fk_returns_target(self):
        for x in range(80, 230, 15):
            for y in range(-80, 81, 20):
                for z in range(-150, 31, 20):
                    try:
                        angles = leg_ik(x, y, z, *L)
                    except UnreachableError:
                        continue
                    self.assertPointAlmostEqual(leg_fk(angles, *L), (x, y, z))

    def test_known_pose(self):
        # Fémur horizontal, tibia vertical : pied à 130 mm, 120 mm sous la coxa.
        angles = leg_ik(130.0, 0.0, -120.0, *L)
        self.assertAlmostEqual(angles.coxa, 0.0)
        self.assertAlmostEqual(angles.femur, 0.0)
        self.assertAlmostEqual(angles.tibia, -math.pi / 2)

    def test_knee_up_solution(self):
        angles = leg_ik(140.0, 0.0, -90.0, *L)
        self.assertGreater(angles.femur, math.atan2(-90.0, 90.0))
        self.assertLess(angles.tibia, 0.0)

    def test_coxa_follows_foot_direction(self):
        angles = leg_ik(100.0, 100.0, -90.0, *L)
        self.assertAlmostEqual(angles.coxa, math.pi / 4)

    def test_out_of_reach(self):
        with self.assertRaises(UnreachableError):
            leg_ik(400.0, 0.0, -50.0, *L)
        with self.assertRaises(UnreachableError):
            leg_ik(55.0, 0.0, 0.0, *L)

    def test_fk_of_zero_angles(self):
        self.assertPointAlmostEqual(leg_fk(LegAngles(0, 0, 0), *L), (250.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
