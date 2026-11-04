"""Déplacements autonomes : aller à un point en contournant les obstacles, ou
suivre ce qui est devant le capteur (une personne, par exemple).

Le robot marche dans toutes les directions, mais le capteur ne voit que
devant (±60° de balayage) : le corps se tourne donc toujours vers la
direction suivie, pour regarder où il va.

Aller à un point : parmi les directions possibles (tous les 10°), on prend
la plus proche de la direction du but dont le chemin est libre sur ~30 cm
(mesures récentes du capteur, mémorisées sur le sol). Si aucune ne l'est, le
robot tourne sur place pour chercher un passage.

Suivre : le point mesuré le plus proche devant (moins de 1,2 m, mesures de
moins d'une seconde) est la cible ; le robot se tourne vers elle et garde
~40 cm entre elle et son centre (il recule si elle s'approche trop).
"""

import math

from .gait import GaitCommand

DEG = math.pi / 180.0

ARRIVE = 30.0  # mm : but atteint
LOOKAHEAD = 300.0  # mm de chemin libre exigé devant
STEP = 10  # deg entre deux directions essayées
SIDE = 90  # deg : directions essayées de chaque côté du corps
SEE = 50.0 * DEG  # plus loin sur le côté, le capteur ne voit pas : tourner d'abord
YAW_GAIN = 1.5  # rad/s par rad d'écart
MIN_SPEED = 20.0  # mm/s près du but
FOLLOW_DIST = 400.0  # mm entre le centre du robot et la cible suivie
FOLLOW_RANGE = 1200.0  # mm : plus loin, ce n'est plus la cible
FOLLOW_SIDE = 70.0 * DEG
FOLLOW_FRESH = 1.0  # s : âge maximal des mesures utilisées pour suivre
FOLLOW_DEAD = 40.0  # mm : écart de distance toléré sans bouger
FOLLOW_TRACK = 250.0  # mm : la cible ne saute pas plus loin entre deux mesures
FOLLOW_LOST = 2.5  # s sans la revoir : on prend ce qui est le plus proche devant
STUCK_TIME = 10.0  # s sans se rapprocher du but : on abandonne
STUCK_GAIN = 30.0  # mm à gagner dans ce temps
REPLAN = 0.1  # s : le chemin libre n'est recherché que toutes les 0,1 s (temps de calcul sur l'ESP32)


class Navigator:
    def __init__(self):
        self.mode = None  # None, "goto" ou "follow"
        self.goal = None
        self.speed = 60.0  # mm/s
        self.yaw_rate = 0.6  # rad/s
        self._last = 0.0  # dernière direction choisie (stabilité)
        self.target = None  # suivi : position de la cible sur le sol
        self._seen = 0.0  # s depuis la dernière mesure de la cible
        self._best = None  # aller à un point : meilleure distance au but
        self._since_best = 0.0
        self._plan, self._plan_age = None, 0.0

    @property
    def active(self):
        return self.mode is not None

    def goto(self, x, y, speed=None):
        self.mode, self.goal = "goto", (x, y)
        if speed:
            self.speed = speed
        self._last = 0.0
        self._best = None
        self._since_best = 0.0
        self._plan, self._plan_age = None, 0.0

    def follow(self, speed=None):
        self.mode, self.goal = "follow", None
        self.target = None
        if speed:
            self.speed = speed

    def stop(self):
        self.mode, self.goal = None, None

    def status(self):
        if self.mode == "goto":
            return "NAV goto %d %d" % (round(self.goal[0]), round(self.goal[1]))
        return "NAV " + (self.mode or "off")

    def command(self, pose, scanner, dt=0.02):
        """Consigne de marche pour ce pas de temps. Retourne (consigne, événement)
        où l'événement vaut "arrived" quand le but est atteint, "stuck" quand
        le robot n'arrive plus à s'en approcher, sinon None."""
        if self.mode == "goto":
            return self._goto(pose, scanner, dt)
        if self.mode == "follow":
            return self._follow(pose, scanner, dt), None
        return GaitCommand(), None

    # --- interne ----------------------------------------------------------

    def _goto(self, pose, scanner, dt):
        x, y, th = pose
        gx, gy = self.goal
        dx, dy = gx - x, gy - y
        dist = math.sqrt(dx * dx + dy * dy)
        if dist < ARRIVE:
            self.stop()
            return GaitCommand(), "arrived"
        # But collé à un obstacle, ou aucun passage : on ne tourne pas en rond indéfiniment.
        if self._best is None or dist < self._best - STUCK_GAIN:
            self._best, self._since_best = dist, 0.0
        else:
            self._since_best += dt
            if self._since_best > STUCK_TIME:
                self.stop()
                return GaitCommand(), "stuck"
        c, s = math.cos(th), math.sin(th)
        bearing = math.atan2(-s * dx + c * dy, c * dx + s * dy)  # but vu du corps
        speed = max(MIN_SPEED, min(self.speed, dist))
        look = min(LOOKAHEAD, dist + 50.0)
        # Directions de la plus proche du but à la plus éloignée ; à écart
        # égal, celle qu'on suivait déjà (pas d'hésitation gauche-droite).
        b = bearing / DEG
        self._plan_age += dt
        if self._plan is not None and self._plan_age < REPLAN:
            return self._plan, None
        self._plan_age = 0.0
        self._plan = self._choose(scanner, pose, b, bearing, speed, look)
        return self._plan, None

    def _choose(self, scanner, pose, b, bearing, speed, look):
        options = [a for a in range(-SIDE, SIDE + 1, STEP)]
        options.sort(key=lambda a: abs(a - b) + 0.3 * abs(a - self._last))
        if abs(b) <= SIDE:
            options.insert(0, b)
        for a in options:
            if not _blocked(scanner, pose, a * DEG, speed, look):
                self._last = a
                return self._walk(a * DEG, speed)
        # Rien de libre devant : tourner sur place, du côté du but, pour chercher un passage.
        turn = self.yaw_rate if (bearing if abs(bearing) > 1e-3 else self._last) >= 0 else -self.yaw_rate
        return GaitCommand(0.0, 0.0, turn)

    def _walk(self, a, speed):
        yaw = max(-self.yaw_rate, min(self.yaw_rate, YAW_GAIN * a))
        if abs(a) > SEE:
            return GaitCommand(0.0, 0.0, yaw)  # le capteur ne voit pas par là : tourner d'abord
        k = math.cos(a)  # plus lent tant que le corps n'est pas tourné vers la direction
        return GaitCommand(speed * k * math.cos(a), speed * k * math.sin(a), yaw)

    def _follow(self, pose, scanner, dt):
        self._seen += dt
        if self._seen > FOLLOW_LOST:
            self.target = None
        found = _nearest_ahead(scanner, pose, self.target)
        if found is None:
            return GaitCommand()  # plus rien devant : attendre (le capteur balaye)
        d, a, wx, wy = found
        self.target = (wx, wy)
        self._seen = 0.0
        yaw = max(-self.yaw_rate, min(self.yaw_rate, YAW_GAIN * a))
        err = d - FOLLOW_DIST
        if abs(err) < FOLLOW_DEAD or abs(a) > SEE:
            return GaitCommand(0.0, 0.0, yaw)
        v = max(-self.speed / 2, min(self.speed, 1.2 * (err - FOLLOW_DEAD if err > 0 else err + FOLLOW_DEAD)))
        return GaitCommand(v * math.cos(a), v * math.sin(a), yaw)


def _blocked(scanner, pose, a, speed, look):
    if scanner is None or not scanner.enabled:
        return False
    return scanner.path_blocked(pose, GaitCommand(speed * math.cos(a), speed * math.sin(a), 0.0), look)


def _nearest_ahead(scanner, pose, near=None):
    """(distance au centre, direction vue du corps, x, y) du point récent le
    plus proche devant ; s'il y a une cible `near`, seulement autour d'elle."""
    if scanner is None or not scanner.enabled:
        return None
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    best = None
    for px, py, t in scanner.points:
        if scanner.time - t > FOLLOW_FRESH:
            continue
        if near is not None and (px - near[0]) ** 2 + (py - near[1]) ** 2 > FOLLOW_TRACK * FOLLOW_TRACK:
            continue
        dx, dy = px - x, py - y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        d = math.sqrt(bx * bx + by * by)
        a = math.atan2(by, bx)
        if d < FOLLOW_RANGE and abs(a) < FOLLOW_SIDE and (best is None or d < best[0]):
            best = (d, a, px, py)
    return best
