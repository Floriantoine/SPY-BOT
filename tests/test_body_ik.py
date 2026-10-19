import math
import unittest

from hexapod.body_ik import (
    BodyPose,
    body_to_leg_frame,
    foot_in_body,
    leg_to_body_frame,
    rotation_matrix,
)
from hexapod.config import default_config


def apply(r, p):
    return tuple(sum(r[i][j] * p[j] for j in range(3)) for i in range(3))


class BodyIKTest(unittest.TestCase):
    def setUp(self):
        self.config = default_config()

    def test_neutral_pose_is_identity(self):
        p = (120.0, -30.0, -90.0)
        self.assertEqual(foot_in_body(p, BodyPose()), p)

    def test_raising_body_lowers_feet(self):
        p = foot_in_body((100.0, 0.0, -90.0), BodyPose(z=20.0))
        self.assertAlmostEqual(p[2], -110.0)

    def test_rotation_is_orthonormal(self):
        r = rotation_matrix(0.2, -0.3, 0.5)
        for i in range(3):
            for j in range(3):
                dot = sum(r[i][k] * r[j][k] for k in range(3))
                self.assertAlmostEqual(dot, 1.0 if i == j else 0.0)

    def test_moving_body_back_onto_foot(self):
        # Le pied vu depuis le corps déplacé, renvoyé dans le monde, redonne le pied.
        pose = BodyPose(10, -5, 15, 0.1, -0.2, 0.3)
        world = (130.0, 70.0, -90.0)
        local = foot_in_body(world, pose)
        r = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
        back = apply(r, local)
        back = (back[0] + pose.x, back[1] + pose.y, back[2] + pose.z)
        for a, b in zip(back, world):
            self.assertAlmostEqual(a, b)

    def test_neutral_foot_points_straight_out_of_leg(self):
        c = self.config
        for leg in c.legs:
            x, y, z = body_to_leg_frame(c.neutral_foot(leg), leg)
            self.assertAlmostEqual(x, c.stance_radius)
            self.assertAlmostEqual(y, 0.0)
            self.assertAlmostEqual(z, -c.stance_height)

    def test_leg_frame_round_trip(self):
        leg = self.config.leg("LR")
        p = (-150.0, 120.0, -80.0)
        q = leg_to_body_frame(body_to_leg_frame(p, leg), leg)
        for a, b in zip(p, q):
            self.assertAlmostEqual(a, b)


if __name__ == "__main__":
    unittest.main()
