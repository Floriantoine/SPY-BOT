import unittest

from hexapod.gait import QUAD_ORDER, TRIPOD_A, TRIPOD_B, WAVE_ORDER, QuadGait, TripodGait, WaveGait
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


class WaveGaitTest(unittest.TestCase):
    def test_one_leg_at_a_time_in_order(self):
        gait = WaveGait(period=2.0)
        order = []
        for _ in range(400):
            gait.advance(0.01, True)
            up = [n for n in WAVE_ORDER if gait.swing(n) not in (None, 0.0)]
            self.assertLessEqual(len(up), 1)
            self.assertGreaterEqual(len(gait.feet_on_ground()), 5)
            if up and (not order or order[-1] != up[0]):
                order.append(up[0])
        self.assertEqual(tuple(order[:6]), WAVE_ORDER)
        self.assertEqual(sorted(WAVE_ORDER), sorted(TRIPOD_A + TRIPOD_B))

    def test_stops_with_all_feet_down(self):
        for start in (0.0, 0.07, 0.31, 0.95):
            gait = WaveGait(period=2.0)
            gait.advance(start * 2.0, True)
            for _ in range(60):
                gait.advance(0.013, False)
            self.assertTrue(gait.at_rest)
            self.assertEqual(len(gait.feet_on_ground()), 6)
            phase = gait.phase
            gait.advance(0.1, False)
            self.assertEqual(gait.phase, phase)

    def test_each_foot_pushes_five_sixths_of_the_cycle(self):
        gait = WaveGait(period=3.0)
        self.assertAlmostEqual(gait.stance_time, 2.5)
        gait.phase = 0.5  # RF (4e) commence son pas
        self.assertAlmostEqual(gait.swing("RF"), 0.0)
        self.assertAlmostEqual(gait.time_to_lift("RM"), 0.5)
