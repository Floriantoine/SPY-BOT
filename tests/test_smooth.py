"""Marche plus douce : poser du pied, démarches, décalage du corps, servos."""

import math
import unittest

from hexapod import GaitCommand, default_config
from hexapod.robot import Hexapod

DT = 0.02


def walk(robot, command, seconds, watch=None):
    robot.command = command
    for _ in range(int(round(seconds / DT))):
        robot.update(DT)
        if watch:
            watch()


class SoftTouchdownTest(unittest.TestCase):
    def check(self, legs):
        r = Hexapod(default_config())
        if legs == 4:
            r.set_legs(4)
            walk(r, GaitCommand(), 1.5)
        h = r.gait.step_height
        prev = {}
        seen = {"peak": 0.0, "step": 0.0, "landings": 0}

        def watch():
            for n in r.walking_legs():
                z = r.offsets[n][2]
                was = prev.get(n, 0.0)
                if (z == 0.0) != (was == 0.0):
                    # Décollage ou poser : presque sans vitesse verticale.
                    seen["step"] = max(seen["step"], abs(z - was))
                    seen["landings"] += z == 0.0
                seen["peak"] = max(seen["peak"], z)
                prev[n] = z

        walk(r, GaitCommand(60.0, 20.0, 0.2), 12, watch)
        self.assertGreater(seen["landings"], 10)
        self.assertGreater(seen["peak"], 0.97 * h)  # même hauteur qu'avant au milieu du pas
        self.assertLessEqual(seen["peak"], h + 1e-9)
        # Avec sin(pi s), le dernier tick descendait de ~13 % de la hauteur (6 pattes).
        self.assertLess(seen["step"], 0.03 * h)

    def test_six_legs(self):
        self.check(6)

    def test_four_legs(self):
        self.check(4)


if __name__ == "__main__":
    unittest.main()
