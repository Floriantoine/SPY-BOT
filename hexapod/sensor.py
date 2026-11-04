"""Capteur de distance avant : VL53L1X sur un servo de balayage (pan).

Le capteur tourne sur un servo à axe vertical posé sur la plaque du dessus.
`Scanner` le fait balayer pas à pas, garde les mesures récentes (dans le
repère du sol, pour qu'elles restent justes quand le robot bouge) et dit si
un obstacle barre le chemin demandé : le contrôleur coupe alors l'avance,
mais laisse tourner et reculer.

Deux sources de mesure, même interface `read(x, y, heading)` (position et
direction du capteur dans le repère du sol, ignorées par le vrai capteur) :
  - `VL53L1X` : le vrai capteur, sur le bus I²C des PCA9685 (adresse 0x29) ;
  - `SimRanger` : capteur simulé, qui vise des obstacles virtuels (cylindres).

Le pilote VL53L1X suit la séquence de la bibliothèque ST « ULD ». Il n'a pas
encore été essayé sur le robot.
"""

import math

DEG = math.pi / 180.0

# Montage (repère corps : x avant, y gauche, mm). Axe du servo de balayage en
# x = 8 ; la puce est ~8 mm devant l'axe et tourne avec lui.
PAN_AXIS_X = 8.0
CHIP_OFFSET = 8.0
CHIP_X = PAN_AXIS_X + CHIP_OFFSET
PAN_SERVO = (1, 11)  # carte 0x41, canal libre (à confirmer au câblage)
PAN_DIRECTION = 1  # +1 : servo qui monte = capteur qui tourne vers la gauche
# Impulsions bornées à 800-2000 µs (voir PCA9685Driver) : ±60° au plus.
PAN_LIMIT = 60.0  # deg
SCAN_STEP = 5.0  # deg entre deux mesures
SCAN_PERIOD = 0.05  # s entre deux mesures (si le capteur suit)
SCAN_TIMEOUT = 0.25  # s : pas de mesure prête -> on passe à l'angle suivant
WALK_HALF = 40.0  # deg : en marche, balayage resserré autour de la direction suivie
MIN_RANGE = 120.0  # mm de la puce : plus près, c'est une patte du robot
MAX_RANGE = 4000.0  # mm, mode longue distance du VL53L1X
POINT_LIFE = 4.0  # s : une mesure plus vieille est oubliée
FOOT_RADIUS = 160.0  # mm : cercle des pieds (156) autour du centre du corps, + marge
STOP_DEFAULT = 80.0  # mm de marge entre les pieds et l'obstacle, le long du chemin
STOP_RANGE = (30.0, 300.0)


class Scanner:
    def __init__(self, ranger=None, driver=None, servo=PAN_SERVO):
        self.ranger = ranger
        self.driver = driver
        self.servo = servo
        self.enabled = False
        self.stop_dist = STOP_DEFAULT
        self.angle = 0.0  # deg, + = vers la gauche
        self.direction = 1
        self.wait = 0.0
        self.time = 0.0
        self.points = []  # (x, y, instant) dans le repère du sol
        self.blocked = False

    @property
    def present(self):
        return self.ranger is not None

    def status(self):
        return "SENSOR present %d on %d stop %d" % (self.present, self.enabled, self.stop_dist)

    def enable(self, on):
        self.enabled = bool(on) and self.present
        self.points = []
        self.blocked = False
        self.wait = 0.0
        if self.enabled:
            self.angle = 0.0
            self._point_servo()

    def tick(self, dt, pose, wanted):
        """Avance le balayage. `pose` = (x, y, cap) du robot dans le repère du
        sol, `wanted` = consigne de marche demandée. Retourne les lignes à
        envoyer au PC (mesures `S angle distance`, 0 = rien en vue)."""
        self.time += dt
        if not self.enabled:
            return []
        self.wait += dt
        if self.wait < SCAN_PERIOD:
            return []
        x, y, th = pose
        heading = th + self.angle * DEG
        cx = x + math.cos(th) * PAN_AXIS_X + math.cos(heading) * CHIP_OFFSET
        cy = y + math.sin(th) * PAN_AXIS_X + math.sin(heading) * CHIP_OFFSET
        d = self.ranger.read(cx, cy, heading)
        if d is None and self.wait < SCAN_TIMEOUT:
            return []  # mesure pas encore prête
        lines = []
        if d is not None:
            seen = MIN_RANGE <= d <= MAX_RANGE
            lines.append("S %d %d" % (round(self.angle), round(d) if seen else 0))
            if seen:
                self.points.append((cx + math.cos(heading) * d, cy + math.sin(heading) * d, self.time))
        self.points = [p for p in self.points if self.time - p[2] < POINT_LIFE]
        self.wait = 0.0
        self._step(wanted)
        self._point_servo()
        return lines

    def check(self, pose, wanted):
        """True si un obstacle est dans le couloir de la marche demandée."""
        self.blocked = self.enabled and self._in_path(pose, wanted)
        return self.blocked

    # --- interne ----------------------------------------------------------

    def _in_path(self, pose, wanted):
        """Suit la trajectoire de la consigne (arc de cercle si le robot tourne
        en avançant) sur `stop_dist` mm : bloqué si un point mesuré entre dans
        le cercle des pieds en chemin. Un point qui s'éloigne ne bloque pas :
        on peut toujours se dégager d'un obstacle déjà tout près."""
        speed = math.sqrt(wanted.vx * wanted.vx + wanted.vy * wanted.vy)
        if speed < 1.0 or not self.points:
            return False
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        near = []
        reach = FOOT_RADIUS + self.stop_dist + 10.0
        for px, py, _ in self.points:
            dx, dy = px - x, py - y
            bx, by = c * dx + s * dy, -s * dx + c * dy  # dans le repère corps
            d2 = bx * bx + by * by
            if d2 < reach * reach:
                near.append((bx, by, d2))
        if not near:
            return False
        w = wanted.yaw_rate
        horizon = min(3.0, self.stop_dist / speed)
        n = max(2, int(self.stop_dist / 15.0))
        r2 = FOOT_RADIUS * FOOT_RADIUS
        for i in range(1, n + 1):
            t = horizon * i / n
            if abs(w) < 1e-4:
                cx, cy = wanted.vx * t, wanted.vy * t
            else:
                a = w * t
                cx = (wanted.vx * math.sin(a) + wanted.vy * (math.cos(a) - 1.0)) / w
                cy = (wanted.vx * (1.0 - math.cos(a)) + wanted.vy * math.sin(a)) / w
            for bx, by, d2 in near:
                e2 = (bx - cx) ** 2 + (by - cy) ** 2
                if e2 < r2 and e2 < d2 - 25.0:
                    return True
        return False

    def _step(self, wanted):
        lo, hi = -PAN_LIMIT, PAN_LIMIT
        if wanted.vx > 0:
            # En marche vers l'avant : balayage resserré autour du cap suivi.
            centre = math.atan2(wanted.vy, wanted.vx) / DEG
            centre = max(-PAN_LIMIT + WALK_HALF, min(PAN_LIMIT - WALK_HALF, centre))
            lo, hi = centre - WALK_HALF, centre + WALK_HALF
        a = self.angle + self.direction * SCAN_STEP
        if a > hi:
            self.direction = -1
            a = max(lo, min(self.angle - SCAN_STEP, hi))
        elif a < lo:
            self.direction = 1
            a = min(hi, max(self.angle + SCAN_STEP, lo))
        self.angle = a

    def _point_servo(self):
        if self.driver is not None:
            board, channel = self.servo
            self.driver.set_angle(board, channel, 90.0 + PAN_DIRECTION * self.angle)


class SimRanger:
    """Capteur simulé : distance au premier obstacle virtuel dans le faisceau.

    Le faisceau (27°, réduit à ~15° par le ROI) est approché par 5 rayons ;
    le capteur rend le plus proche, comme le vrai."""

    BEAM = 7.5 * DEG  # demi-angle

    def __init__(self, obstacles=None):
        self.obstacles = list(obstacles or [])  # (x, y, rayon) en mm, repère du sol

    def read(self, x, y, heading):
        best = None
        for k in (-1.0, -0.5, 0.0, 0.5, 1.0):
            a = heading + k * self.BEAM
            ux, uy = math.cos(a), math.sin(a)
            for ox, oy, r in self.obstacles:
                d = _ray_circle(x, y, ux, uy, ox, oy, r)
                if d is not None and (best is None or d < best):
                    best = d
        if best is None or best > MAX_RANGE:
            return MAX_RANGE + 1  # rien en vue
        return best


def _ray_circle(x, y, ux, uy, cx, cy, r):
    fx, fy = x - cx, y - cy
    b = fx * ux + fy * uy
    c = fx * fx + fy * fy - r * r
    disc = b * b - c
    if disc < 0:
        return None
    t = -b - math.sqrt(disc)
    if t < 0:
        t = -b + math.sqrt(disc)
        if t < 0:
            return None
        return 0.0  # départ dans l'obstacle
    return t


# Configuration par défaut du VL53L1X (registres 0x2D à 0x87), tirée de la
# bibliothèque ST « ULD » (VL51L1X_DEFAULT_CONFIGURATION).
_DEFAULT_CONFIG = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xDB, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40,
])


class VL53L1X:
    """Pilote minimal du VL53L1X (mesure continue, non bloquante)."""

    ADDRESS = 0x29
    MODEL_ID = 0xEACC

    def __init__(self, i2c, address=ADDRESS):
        import time

        self.i2c = i2c
        self.address = address
        if self._read16(0x010F) != self.MODEL_ID:
            raise OSError("pas un VL53L1X")
        for _ in range(100):  # attente de la fin du démarrage interne
            if self._read8(0x00E5) & 0x01:
                break
            time.sleep_ms(2)
        self._write(0x002D, _DEFAULT_CONFIG)
        # Première mesure pour finir l'initialisation (séquence ULD).
        for _ in range(100):
            if self._ready():
                break
            time.sleep_ms(2)
        self._write8(0x0086, 0x01)  # efface l'interruption
        self._write8(0x0087, 0x00)  # arrêt
        self._write8(0x0008, 0x09)  # VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND
        self._write8(0x000B, 0x00)
        # Champ de vue réduit de 27° à ~15° (ROI de 4×4 SPAD au centre) : un
        # obstacle à côté du chemin paraît moins « étalé ». Portée un peu réduite.
        self._write8(0x0080, 0x33)
        self._write8(0x0087, 0x40)  # mesure continue

    @classmethod
    def probe(cls, i2c):
        """Le capteur s'il répond, sinon None (absent ou mal câblé)."""
        try:
            if cls.ADDRESS not in i2c.scan():
                return None
            return cls(i2c)
        except OSError:
            return None

    def read(self, x=0.0, y=0.0, heading=0.0):
        """Distance en mm si une mesure est prête, sinon None. Une mesure
        invalide (rien en vue, signal trop faible) donne MAX_RANGE + 1."""
        try:
            if not self._ready():
                return None
            status = self._read8(0x0089) & 0x1F
            d = self._read16(0x0096)
            self._write8(0x0086, 0x01)  # efface l'interruption : mesure suivante
        except OSError:
            return None
        return float(d) if status == 9 else MAX_RANGE + 1

    def _ready(self):
        polarity = 0 if self._read8(0x0030) & 0x10 else 1
        return (self._read8(0x0031) & 0x01) == polarity

    def _write(self, reg, data):
        self.i2c.writeto(self.address, bytes([reg >> 8, reg & 0xFF]) + data)

    def _write8(self, reg, value):
        self._write(reg, bytes([value]))

    def _read(self, reg, n):
        self.i2c.writeto(self.address, bytes([reg >> 8, reg & 0xFF]))
        return self.i2c.readfrom(self.address, n)

    def _read8(self, reg):
        return self._read(reg, 1)[0]

    def _read16(self, reg):
        b = self._read(reg, 2)
        return (b[0] << 8) | b[1]
