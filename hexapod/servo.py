"""Conversion angles IK -> angles servo, et pilotes PCA9685."""

import math

from .config import I2C_ADDRESSES

DEG_PER_RAD = 180.0 / math.pi


def ik_to_servo_deg(angle_rad, joint):
    """Angle servo (0..180°) pour un angle articulaire IK, borné aux limites."""
    deg = 90.0 + joint.direction * (angle_rad * DEG_PER_RAD - joint.zero_deg) + joint.trim_deg
    return max(joint.min_deg, min(joint.max_deg, deg))


class DryRunDriver:
    """Pilote factice : mémorise les angles, utile pour les tests et la simulation."""

    def __init__(self):
        self.angles = {}

    def set_angle(self, board, channel, deg):
        self.angles[(board, channel)] = deg


class PCA9685:
    """Une carte PCA9685 (16 sorties PWM) sur un bus I²C MicroPython."""

    MODE1 = 0x00
    PRESCALE = 0xFE
    LED0_ON_L = 0x06

    def __init__(self, i2c, address=0x40, freq=50):
        self.i2c = i2c
        self.address = address
        self.freq = freq
        self._write(self.MODE1, 0x00)
        self.set_freq(freq)

    def _write(self, reg, value):
        self.i2c.writeto_mem(self.address, reg, bytes([value]))

    def _read(self, reg):
        return self.i2c.readfrom_mem(self.address, reg, 1)[0]

    def set_freq(self, freq):
        import time

        prescale = int(25000000.0 / 4096.0 / freq + 0.5) - 1
        old = self._read(self.MODE1)
        self._write(self.MODE1, (old & 0x7F) | 0x10)  # veille pour changer le prescaler
        self._write(self.PRESCALE, prescale)
        self._write(self.MODE1, old)
        time.sleep_us(500)
        self._write(self.MODE1, old | 0xA1)  # redémarrage + auto-incrément
        self.freq = freq

    def set_pulse_us(self, channel, us):
        off = int(us * self.freq * 4096 / 1000000)
        data = bytes([0, 0, off & 0xFF, off >> 8])
        self.i2c.writeto_mem(self.address, self.LED0_ON_L + 4 * channel, data)

    def off(self, channel):
        # Bit « full off » : le servo n'est plus alimenté en consigne.
        self.i2c.writeto_mem(self.address, self.LED0_ON_L + 4 * channel, bytes([0, 0, 0, 0x10]))


class PCA9685Driver:
    """Pilote réel : ESP32 sous MicroPython avec deux PCA9685 sur le même bus I²C.

    Par défaut : SDA = GPIO21, SCL = GPIO22, cartes en 0x40 (pattes droites)
    et 0x41 (pattes gauches). Impulsions de hexapod-v2-7697 pour des MG92B :
    1400 µs au milieu, 10 µs par degré, bornées à 800-2000 µs.
    """

    def __init__(self, i2c=None, addresses=I2C_ADDRESSES, sda=21, scl=22,
                 center_us=1400, us_per_deg=10.0, min_us=800, max_us=2000):
        if i2c is None:
            from machine import I2C, Pin

            i2c = I2C(0, sda=Pin(sda), scl=Pin(scl), freq=100000)  # 100 kHz : plus tolérant aux parasites des servos
        # Une carte absente ne bloque pas le démarrage : ses servos sont
        # ignorés et `missing` la signale au PC.
        self.boards = []
        self.missing = []
        self.errors = 0  # écritures I²C échouées
        for a in addresses:
            try:
                self.boards.append(PCA9685(i2c, a))
            except OSError:
                self.boards.append(None)
                self.missing.append(a)
        self.center_us = center_us
        self.us_per_deg = us_per_deg
        self.min_us = min_us
        self.max_us = max_us

    def set_angle(self, board, channel, deg):
        us = self.center_us + (deg - 90.0) * self.us_per_deg
        us = max(self.min_us, min(self.max_us, us))
        if self.boards[board]:
            try:
                self.boards[board].set_pulse_us(channel, us)
            except OSError:
                # Écriture I²C perdue (parasites des servos) : la consigne
                # suivante, 20 ms plus tard, la remplace.
                self.errors += 1

    def relax(self):
        for board in self.boards:
            if not board:
                continue
            for channel in range(16):
                try:
                    board.off(channel)
                except OSError:
                    pass
