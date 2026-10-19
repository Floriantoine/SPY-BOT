"""Conversion angles IK -> angles servo, et pilotes PCA9685."""

import math


def ik_to_servo_deg(angle_rad, joint):
    """Angle servo (0..180°) pour un angle articulaire IK, borné aux limites."""
    deg = 90.0 + joint.direction * (math.degrees(angle_rad) - joint.zero_deg) + joint.trim_deg
    return max(joint.min_deg, min(joint.max_deg, deg))


class DryRunDriver:
    """Pilote factice : mémorise les angles, utile pour les tests et la simulation."""

    def __init__(self):
        self.angles = {}

    def set_angle(self, board, channel, deg):
        self.angles[(board, channel)] = deg


class PCA9685Driver:
    """Pilote réel via `adafruit-circuitpython-servokit` (Raspberry Pi).

    pip install adafruit-circuitpython-servokit
    """

    def __init__(self, addresses=(0x40, 0x41), pulse_range=(500, 2500)):
        from adafruit_servokit import ServoKit  # import tardif : optionnel hors robot

        self.kits = [ServoKit(channels=16, address=a) for a in addresses]
        for kit in self.kits:
            for servo in kit.servo:
                servo.set_pulse_width_range(*pulse_range)

    def set_angle(self, board, channel, deg):
        self.kits[board].servo[channel].angle = deg
