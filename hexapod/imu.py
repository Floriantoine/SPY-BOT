"""Centrale inertielle MPU6050 : inclinaison du corps (roulis, tangage).

Le MPU6050 est vissé sur plaque_bas, à l'arrière droite du corps (x -38,5 à
-17,5, y -30 à -14), sur le même bus I²C que les PCA9685 (AD0 à la masse :
adresse 0x68), alimenté en 3,3 V par l'ESP32.

Filtre complémentaire : le gyroscope suit les mouvements rapides, l'
accéléromètre (direction de la pesanteur) corrige la dérive lente. Le
biais du gyroscope est mesuré au démarrage, robot immobile.

Convention : mêmes angles que BodyPose (roll autour de x avant, pitch autour
de y gauche, en radians). Sur sol plat, une POSE de roulis +5° doit se lire
+5° ici ; sinon, corriger IMU_AXES (orientation de la carte sur le corps).

Le pilote n'a pas encore été essayé sur le robot.
"""

import math

# Axes de la carte vus du corps : pour chaque axe du corps (x avant, y gauche,
# z haut), (indice de l'axe de la carte, signe). Carte posée à plat, composants
# vers le haut, flèche X vers l'avant : ((0, 1), (1, 1), (2, 1)).
IMU_AXES = ((0, 1), (1, 1), (2, 1))
ADDRESS = 0x68
ACCEL_TRUST = 0.02  # part de l'accéléromètre à chaque mesure (à 50 Hz : ~1 s)
ACCEL_OK = (0.8, 1.2)  # g : en dehors (choc, pas), on ne croit pas l'accéléromètre
BIAS_TIME = 1.0  # s de mesure du biais du gyroscope au démarrage
LEVEL_GAIN = 2.0  # 1/s : vitesse de la correction d'assiette
LEVEL_LIMIT = 10.0 * math.pi / 180  # rad : correction maximale


class Attitude:
    """Roulis et tangage estimés à partir des mesures brutes (g et rad/s)."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.ready = False  # biais mesuré, angles initialisés
        self._bias = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_t = 0.0

    def update(self, dt, accel, gyro):
        ax, ay, az = accel
        if not self.ready:
            # Robot immobile : on moyenne le gyroscope, l'accéléromètre donne l'inclinaison.
            self._bias = [b + g for b, g in zip(self._bias, gyro)]
            self._bias_n += 1
            self._bias_t += dt
            if self._bias_t >= BIAS_TIME:
                self._bias = [b / self._bias_n for b in self._bias]
                self.roll, self.pitch = _accel_angles(ax, ay, az)
                self.ready = True
            return
        gx, gy = gyro[0] - self._bias[0], gyro[1] - self._bias[1]
        roll = self.roll + gx * dt
        pitch = self.pitch + gy * dt
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if ACCEL_OK[0] < g < ACCEL_OK[1]:
            ar, ap = _accel_angles(ax, ay, az)
            roll += (ar - roll) * ACCEL_TRUST
            pitch += (ap - pitch) * ACCEL_TRUST
        self.roll, self.pitch = roll, pitch


def _accel_angles(ax, ay, az):
    """Roulis et tangage d'après la pesanteur mesurée (repère corps)."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return roll, pitch


def gravity_in_body(roll, pitch):
    """Ce que mesure l'accéléromètre (en g) pour une assiette donnée, immobile."""
    return (-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll))


class Leveler:
    """Auto-équilibrage : incline le corps (POSE) pour qu'il reste à
    l'assiette demandée malgré la pente du sol."""

    def __init__(self):
        self.enabled = False
        self.roll = 0.0  # correction ajoutée à la pose (rad)
        self.pitch = 0.0

    def reset(self):
        self.roll = self.pitch = 0.0

    def relax(self, dt):
        """Hors de la marche (assis, en train de s'asseoir) : la correction
        revient doucement à zéro, sans à-coup."""
        step = LEVEL_LIMIT * dt
        self.roll = max(0.0, self.roll - step) if self.roll > 0 else min(0.0, self.roll + step)
        self.pitch = max(0.0, self.pitch - step) if self.pitch > 0 else min(0.0, self.pitch + step)

    def update(self, dt, measured, wanted):
        """`measured` = (roulis, tangage) lus, `wanted` = assiette voulue."""
        if not self.enabled:
            self.reset()
            return
        k = min(1.0, LEVEL_GAIN * dt)
        self.roll = _clamp(self.roll - (measured[0] - wanted[0]) * k, LEVEL_LIMIT)
        self.pitch = _clamp(self.pitch - (measured[1] - wanted[1]) * k, LEVEL_LIMIT)


def _clamp(v, limit):
    return max(-limit, min(limit, v))


class MPU6050:
    """Pilote minimal : accélérations (g) et vitesses de rotation (rad/s), repère corps."""

    ACCEL_SCALE = 8192.0  # LSB/g à ±4 g
    GYRO_SCALE = 65.5 * 180 / math.pi  # LSB/(rad/s) à ±500 °/s

    def __init__(self, i2c, address=ADDRESS):
        self.i2c = i2c
        self.address = address
        self.i2c.writeto_mem(address, 0x6B, b"\x01")  # réveil, horloge du gyroscope X
        self.i2c.writeto_mem(address, 0x1A, b"\x03")  # filtre passe-bas ~44 Hz (vibrations des servos)
        self.i2c.writeto_mem(address, 0x1B, b"\x08")  # gyroscope ±500 °/s
        self.i2c.writeto_mem(address, 0x1C, b"\x08")  # accéléromètre ±4 g
        self.attitude = Attitude()
        self.errors = 0

    @classmethod
    def probe(cls, i2c):
        try:
            if ADDRESS not in i2c.scan():
                return None
            return cls(i2c)
        except OSError:
            return None

    def read(self):
        """((ax, ay, az), (gx, gy, gz)) dans le repère corps, ou None si la lecture échoue."""
        try:
            b = self.i2c.readfrom_mem(self.address, 0x3B, 14)
        except OSError:
            self.errors += 1
            return None
        raw = [_s16(b[i], b[i + 1]) for i in (0, 2, 4, 8, 10, 12)]
        accel = _to_body([v / self.ACCEL_SCALE for v in raw[:3]])
        gyro = _to_body([v / self.GYRO_SCALE for v in raw[3:]])
        return accel, gyro

    def update(self, dt, pose=None):
        m = self.read()
        if m:
            self.attitude.update(dt, m[0], m[1])


class SimIMU:
    """Centrale simulée : le sol est penché de (`ground_roll`, `ground_pitch`),
    le corps s'incline en plus de la pose commandée."""

    def __init__(self):
        self.ground_roll = 0.0
        self.ground_pitch = 0.0
        self.attitude = Attitude()
        self._prev = None

    def update(self, dt, pose):
        roll = self.ground_roll + pose.roll
        pitch = self.ground_pitch + pose.pitch
        if self._prev is None:
            self._prev = (roll, pitch)
        gyro = ((roll - self._prev[0]) / dt if dt else 0.0, (pitch - self._prev[1]) / dt if dt else 0.0, 0.0)
        self._prev = (roll, pitch)
        self.attitude.update(dt, gravity_in_body(roll, pitch), gyro)


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def _to_body(v):
    return [v[i] * s for i, s in IMU_AXES]
