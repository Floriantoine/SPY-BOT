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
# Pendant cette mesure le robot doit être immobile : si le gyroscope varie de
# plus que ça (~3 °/s), on le bouge, et la mesure recommence.
BIAS_SPREAD = 0.05  # rad/s
LEVEL_GAIN = 2.0  # 1/s : vitesse de la correction d'assiette
LEVEL_LIMIT = 10.0 * math.pi / 180  # rad : correction maximale


class Attitude:
    """Roulis et tangage estimés à partir des mesures brutes (g et rad/s)."""

    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.ready = False  # biais mesuré, angles initialisés
        self.rate = (0.0, 0.0)  # vitesse de roulis et de tangage (rad/s), biais retiré
        self._bias = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_t = 0.0
        self._lo = [0.0, 0.0, 0.0]  # plus petite et plus grande mesure du
        self._hi = [0.0, 0.0, 0.0]  # gyroscope depuis le début de la mesure du biais

    def restart(self):
        """Refait la mesure du biais (IMU CAL) : pas prêt pendant BIAS_TIME, robot immobile."""
        self.ready = False
        self.rate = (0.0, 0.0)
        self._bias = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_t = 0.0

    def update(self, dt, accel, gyro):
        ax, ay, az = accel
        if not self.ready:
            # Robot immobile : on moyenne le gyroscope, l'accéléromètre donne l'inclinaison.
            moved = False
            for i in range(3):
                g = gyro[i]
                if self._bias_n == 0 or g < self._lo[i]:
                    self._lo[i] = g
                if self._bias_n == 0 or g > self._hi[i]:
                    self._hi[i] = g
                if self._hi[i] - self._lo[i] > BIAS_SPREAD:
                    moved = True
            if moved:
                self.restart()  # on a bougé le robot : la mesure recommence
                return
            self._bias = [b + g for b, g in zip(self._bias, gyro)]
            self._bias_n += 1
            self._bias_t += dt
            if self._bias_t >= BIAS_TIME:
                self._bias = [b / self._bias_n for b in self._bias]
                self.roll, self.pitch = _accel_angles(ax, ay, az)
                self.ready = True
            return
        gx, gy, gz = gyro[0] - self._bias[0], gyro[1] - self._bias[1], gyro[2] - self._bias[2]
        # Vitesses du corps -> dérivées du roulis et du tangage (corps penché
        # qui tourne : une part du lacet passe dans le roulis et le tangage).
        sr, cr = math.sin(self.roll), math.cos(self.roll)
        cp = math.cos(self.pitch)
        tp = math.sin(self.pitch) / cp if abs(cp) > 1e-3 else 0.0
        droll = gx + (gy * sr + gz * cr) * tp
        dpitch = gy * cr - gz * sr
        self.rate = (droll, dpitch)
        roll = self.roll + droll * dt
        pitch = self.pitch + dpitch * dt
        g = math.sqrt(ax * ax + ay * ay + az * az)
        if ACCEL_OK[0] < g < ACCEL_OK[1]:
            ar, ap = _accel_angles(ax, ay, az)
            trust = min(1.0, ACCEL_TRUST * dt / 0.02)  # même constante de temps si un tick déborde
            roll += (ar - roll) * trust
            pitch += (ap - pitch) * trust
        self.roll, self.pitch = roll, pitch


def _accel_angles(ax, ay, az):
    """Roulis et tangage d'après la pesanteur mesurée (repère corps)."""
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    return roll, pitch


def euler_rates_to_body(roll, pitch, droll, dpitch, dyaw):
    """Vitesses de rotation (rad/s) dans le repère du corps pour des dérivées
    des angles (lacet, tangage, roulis : ordre ZYX comme BodyPose)."""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    return (droll - dyaw * sp, dpitch * cr + dyaw * sr * cp, -dpitch * sr + dyaw * cr * cp)


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
            self.relax(dt)  # LEVEL 0 : la correction s'en va doucement, sans à-coup
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

    def update(self, dt, pose=None, odom=None, feet=None, swing=()):
        """Mêmes arguments que SimIMU.update (ceux de la simulation sont ignorés)."""
        m = self.read()
        if m:
            self.attitude.update(dt, m[0], m[1])


ROCK_TIME = 0.15  # s : le robot bascule sur ses nouveaux pieds posés en ~0,15 s


class SimIMU:
    """Centrale simulée : le sol est penché de (`ground_roll`, `ground_pitch`),
    et avec `terrain` le robot repose sur ses pieds posés sur le terrain
    accidenté (hexapod/terrain.py) ; le corps s'incline en plus de la pose
    commandée."""

    def __init__(self):
        self.ground_roll = 0.0
        self.ground_pitch = 0.0
        self.terrain = False
        self.slope = (0.0, 0.0)  # pente du sol sous le robot, vue du corps (roulis, tangage)
        self.plane = None  # terrain : plan des pieds posés z = a + b·x + c·y (mm, sol)
        self.air = []  # terrain : hauteur de chaque pied posé au-dessus du terrain
        self._rest = None
        self.attitude = Attitude()
        self._prev = None

    def update(self, dt, pose, odom=None, feet=None, swing=()):
        sr, sp = self.ground_roll, self.ground_pitch
        if self.terrain and odom is not None and feet:
            from .terrain import rest_on

            a, b, c, roll, pitch, self.air = rest_on(feet, odom, swing)
            # Le robot bascule sur ses nouveaux pieds posés en un instant
            # bref, pas d'un coup (souplesse des servos et des pièces).
            # Terrain qu'on vient d'activer : on part d'un sol à plat.
            k = min(1.0, dt / ROCK_TIME) if dt else 0.0
            p = self.plane or (a, 0.0, 0.0)
            self.plane = tuple(u + (v - u) * k for u, v in zip(p, (a, b, c)))
            r0 = self._rest or (0.0, 0.0)
            self._rest = (r0[0] + (roll - r0[0]) * k, r0[1] + (pitch - r0[1]) * k)
            sr += self._rest[0]
            sp += self._rest[1]
        else:
            self.plane, self.air = None, []
            if self._rest:
                # Terrain coupé : le robot revient à plat en douceur.
                k = min(1.0, dt / ROCK_TIME) if dt else 0.0
                r0 = self._rest
                self._rest = (r0[0] * (1 - k), r0[1] * (1 - k))
                if abs(self._rest[0]) + abs(self._rest[1]) < 1e-4:
                    self._rest = None
                else:
                    sr += self._rest[0]
                    sp += self._rest[1]
        self.slope = (sr, sp)
        roll = sr + pose.roll
        pitch = sp + pose.pitch
        yaw = odom[2] if odom is not None else 0.0
        if self._prev is None:
            self._prev = (roll, pitch, yaw)
        if dt:
            # Ce que mesure un vrai gyroscope : les vitesses de rotation dans
            # le repère du corps, y compris le cap qui tourne (lacet).
            dr, dp, dy = (roll - self._prev[0]) / dt, (pitch - self._prev[1]) / dt, (yaw - self._prev[2]) / dt
            gyro = euler_rates_to_body(roll, pitch, dr, dp, dy)
        else:
            gyro = (0.0, 0.0, 0.0)
        self._prev = (roll, pitch, yaw)
        self.attitude.update(dt, gravity_in_body(roll, pitch), gyro)


def _s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


def _to_body(v):
    return [v[i] * s for i, s in IMU_AXES]
