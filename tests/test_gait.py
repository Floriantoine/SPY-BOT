import unittest

from hexapod.gait import QUAD_ORDER, TRIPOD_A, TRIPOD_B, QuadGait, TripodGait
from hexapod.config import default_config


class TripodGaitTest(unittest.TestCase):
    def test_tripods_alternate(self):
        gait = TripodGait(period=1.0)
        gait.phase = 0.25
        self.assertTrue(all(gait.swing(n) == 0.5 for n in TRIPOD_A))
        self.assertTrue(all(gait.swing(n) is None for n in TRIPOD_B))
        gait.phase = 0.75
        self.assertTrue(all(gait.swing(n) is None for n in TRIPOD_A))
        self.assertTrue(all(gait.swing(n) == 0.5 for n in TRIPOD_B))

    def test_three_feet_always_on_ground(self):
        gait = TripodGait(period=1.0)
        for _ in range(200):
            gait.advance(0.013, True)
            self.assertGreaterEqual(len(gait.feet_on_ground()), 3)

    def test_stops_with_all_feet_down(self):
        gait = TripodGait(period=1.0)
        gait.advance(0.3, True)
        for _ in range(50):
            gait.advance(0.05, False)
        self.assertIn(gait.phase, (0.0, 0.5))
        self.assertEqual(len(gait.feet_on_ground()), 6)
        phase = gait.phase
        gait.advance(0.1, False)
        self.assertEqual(gait.phase, phase)


class QuadGaitTest(unittest.TestCase):
    def setUp(self):
        config = default_config()
        self.gait = QuadGait({leg.name: config.neutral_foot(leg) for leg in config.legs}, period=2.0)

    def test_corner_legs_step_one_at_a_time_in_order(self):
        order = []
        for _ in range(400):
            self.gait.advance(0.01, True)
            up = [n for n in QUAD_ORDER if self.gait.swing(n) not in (None, 0.0)]
            self.assertLessEqual(len(up), 1)
            if up and (not order or order[-1] != up[0]):
                order.append(up[0])
        self.assertEqual(tuple(order[:4]), QUAD_ORDER)

    def test_middle_legs_never_step(self):
        for _ in range(400):
            self.gait.advance(0.01, True)
            self.assertIsNone(self.gait.swing("LM"))
            self.assertIsNone(self.gait.swing("RM"))
