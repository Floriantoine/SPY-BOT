import unittest

from hexapod.config import default_config
from hexapod.gait import TRIPOD_A, TRIPOD_B, GaitCommand, TripodGait


class TripodGaitTest(unittest.TestCase):
    def setUp(self):
        self.config = default_config()
        self.neutral = {leg.name: self.config.neutral_foot(leg) for leg in self.config.legs}

    def offsets(self, gait, command):
        return {n: gait.foot_offset(n, p, command) for n, p in self.neutral.items()}

    def test_three_feet_always_on_ground(self):
        gait = TripodGait(period=1.0)
        cmd = GaitCommand(vx=80.0)
        for _ in range(200):
            gait.advance(0.013, cmd)
            grounded = [n for n, o in self.offsets(gait, cmd).items() if o[2] == 0.0]
            self.assertGreaterEqual(len(grounded), 3)

    def test_tripods_alternate(self):
        gait = TripodGait(period=1.0)
        cmd = GaitCommand(vx=80.0)
        gait.phase = 0.25
        o = self.offsets(gait, cmd)
        self.assertTrue(all(o[n][2] > 0 for n in TRIPOD_A))
        self.assertTrue(all(o[n][2] == 0 for n in TRIPOD_B))
        gait.phase = 0.75
        o = self.offsets(gait, cmd)
        self.assertTrue(all(o[n][2] == 0 for n in TRIPOD_A))
        self.assertTrue(all(o[n][2] > 0 for n in TRIPOD_B))

    def test_stance_feet_push_backwards_when_walking_forward(self):
        gait = TripodGait(period=1.0)
        cmd = GaitCommand(vx=80.0)
        gait.phase = 0.6
        before = self.offsets(gait, cmd)["RF"][0]
        gait.phase = 0.9
        after = self.offsets(gait, cmd)["RF"][0]
        self.assertLess(after, before)
        # Vitesse au sol = -vx : 0.3 cycle = 0.3 s -> 24 mm.
        self.assertAlmostEqual(before - after, 24.0)

    def test_trajectory_is_continuous(self):
        gait = TripodGait(period=1.0)
        cmd = GaitCommand(vx=60.0, vy=-20.0, yaw_rate=0.3)
        prev = self.offsets(gait, cmd)
        for _ in range(1000):
            gait.advance(0.002, cmd)
            cur = self.offsets(gait, cmd)
            for n in cur:
                for a, b in zip(prev[n], cur[n]):
                    self.assertLess(abs(a - b), 1.0)
            prev = cur

    def test_turning_in_place_moves_feet_tangentially(self):
        gait = TripodGait(period=1.0)
        cmd = GaitCommand(yaw_rate=0.4)
        for name, p in self.neutral.items():
            sx, sy = gait.stride(p, cmd)
            self.assertAlmostEqual(sx * p[0] + sy * p[1], 0.0)

    def test_stops_with_all_feet_down(self):
        gait = TripodGait(period=1.0)
        gait.advance(0.3, GaitCommand(vx=50.0))
        stop = GaitCommand()
        for _ in range(50):
            gait.advance(0.05, stop)
        self.assertIn(gait.phase, (0.0, 0.5))
        self.assertTrue(all(o[2] == 0.0 for o in self.offsets(gait, stop).values()))
        phase = gait.phase
        gait.advance(0.1, stop)
        self.assertEqual(gait.phase, phase)

    def test_stride_is_capped(self):
        gait = TripodGait(period=1.0, max_stride=40.0)
        sx, sy = gait.stride(self.neutral["RF"], GaitCommand(vx=1000.0))
        self.assertAlmostEqual((sx**2 + sy**2) ** 0.5, 40.0)


if __name__ == "__main__":
    unittest.main()
