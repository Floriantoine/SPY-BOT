"""Rythme des démarches : quelle patte est en l'air, et où elle en est.

Tripode (6 pattes) : deux trépieds alternent ; pendant que l'un pousse au sol
(appui), l'autre se lève et revient vers l'avant (transfert). Trois pieds
touchent donc toujours le sol, ce qui rend la marche statiquement stable.

Vague (6 pattes) : une seule patte en l'air à la fois, 5 pieds au sol (voir
WaveGait) ; plus lente, moins d'effort pour chaque servo.

4 pattes : les pattes des coins se lèvent une à une, les pattes du milieu
restent repliées (voir QuadGait).

Les positions des pieds sont calculées par Hexapod (robot.py) : un pied au
sol recule exactement à la vitesse du robot, un pied en l'air va de là où il
s'est levé jusqu'à son point de pose. Les pieds ne glissent donc jamais,
même quand la consigne change.
"""

import math

TRIPOD_A = ("RF", "LM", "RR")
TRIPOD_B = ("LF", "RM", "LR")
# Marche à 4 pattes : ordre des pas (arrière gauche, avant gauche, arrière
# droite, avant droite) et pattes repliées.
QUAD_ORDER = ("LR", "LF", "RR", "RF")
QUAD_TUCKED = ("LM", "RM")
# Marche « vague » à 6 pattes : ordre des pas, en faisant le tour du corps
# (côté gauche de l'arrière vers l'avant, puis côté droit de l'avant vers
# l'arrière) : la patte en l'air change toujours pour sa voisine.
WAVE_ORDER = ("LR", "LM", "LF", "RF", "RM", "RR")


class GaitCommand:
    """Consigne de déplacement dans le repère corps.

    vx en mm/s vers l'avant, vy en mm/s vers la gauche, yaw_rate en rad/s
    (positif = vers la gauche).
    """

    def __init__(self, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.vx = vx
        self.vy = vy
        self.yaw_rate = yaw_rate

    @property
    def is_zero(self):
        return self.vx == 0 and self.vy == 0 and self.yaw_rate == 0


def ground_velocity(foot, command):
    """Vitesse (mm/s) d'un pied posé, vue du corps : l'inverse du mouvement
    du corps (translation et rotation autour de son centre)."""
    return (-(command.vx - command.yaw_rate * foot[1]), -(command.vy + command.yaw_rate * foot[0]))


class TripodGait:
    def __init__(self, period=1.4, step_height=30.0, max_stride=60.0):
        self.period = period  # durée d'un cycle complet (s)
        self.step_height = step_height  # hauteur de levée du pied (mm)
        self.max_stride = max_stride  # longueur de pas maximale (mm)
        self.phase = 0.0  # 0..1
        self.shift = (0.0, 0.0)  # pas de balancement du corps

    @property
    def stance_time(self):
        """Durée (s) pendant laquelle un pied pousse au sol à chaque cycle."""
        return self.period / 2

    @property
    def at_rest(self):
        """Tous les pieds au sol : on peut s'arrêter là."""
        return self.phase in (0.0, 0.5)

    def reset(self):
        self.phase = 0.0

    def advance(self, dt, moving):
        """Avance la phase. Sans mouvement, s'arrête au prochain instant où
        tous les pieds sont posés (début ou milieu de cycle)."""
        new_phase = self.phase + dt / self.period
        if not moving:
            if self.at_rest:
                return
            new_phase = min(new_phase, 0.5 if self.phase < 0.5 else 1.0)
        self.phase = new_phase % 1.0

    def swing(self, leg_name):
        """Avancement (0..1) du pas de `leg_name`, ou None si le pied est au sol."""
        phase = self.phase if leg_name in TRIPOD_A else (self.phase + 0.5) % 1.0
        return phase / 0.5 if phase < 0.5 else None

    def time_to_lift(self, leg_name):
        """Temps (s) avant que ce pied posé ne se lève."""
        phase = self.phase if leg_name in TRIPOD_A else (self.phase + 0.5) % 1.0
        return (1.0 - phase) * self.period

    def feet_on_ground(self):
        return [n for n in TRIPOD_A + TRIPOD_B if self.swing(n) is None or self.swing(n) == 0.0]


class WaveGait:
    """Marche à 6 pattes, une patte en l'air à la fois (« wave gait ») :
    5 pieds touchent toujours le sol. Le cycle est découpé en 6 parts, une
    par patte (WAVE_ORDER) ; au début de chaque part, les 6 pieds sont au sol.
    Chaque pied pousse pendant 5/6 du cycle : à longueur de pas égale, la
    marche est bien plus lente qu'en tripode."""

    def __init__(self, period=2.8, step_height=30.0, max_stride=60.0):
        self.period = period  # durée d'un cycle complet : chaque patte fait un pas
        self.step_height = step_height
        self.max_stride = max_stride
        self.phase = 0.0
        self.shift = (0.0, 0.0)

    @property
    def swing_part(self):
        return 1.0 / 6

    @property
    def stance_time(self):
        return (1.0 - self.swing_part) * self.period

    @property
    def at_rest(self):
        q = self.phase * 6
        return abs(q - round(q)) < 1e-9

    def reset(self):
        self.phase = 0.0

    def advance(self, dt, moving):
        if not moving and self.at_rest:
            return
        new_phase = self.phase + dt / self.period
        if not moving:
            # Arrêt au prochain sixième : tous les pieds au sol.
            new_phase = min(new_phase, (int(self.phase * 6 + 1e-9) + 1) / 6.0)
        self.phase = new_phase % 1.0
        if abs(self.phase - 1.0) < 1e-9:
            self.phase = 0.0

    def swing(self, leg_name):
        w = self.swing_part
        u = (self.phase - WAVE_ORDER.index(leg_name) / 6) % 1.0
        return u / w if u < w - 1e-9 else None

    def time_to_lift(self, leg_name):
        u = (self.phase - WAVE_ORDER.index(leg_name) / 6) % 1.0
        return (1.0 - u) * self.period

    def feet_on_ground(self):
        return [n for n in WAVE_ORDER if self.swing(n) is None or self.swing(n) == 0.0]


QUAD_SHIFT = 0.35  # part de chaque quart de cycle où le corps se décale, 4 pieds au sol


class QuadGait:
    """Marche à 4 pattes, une patte en l'air à la fois (« creep gait »).

    Le cycle est découpé en 4 quarts, un par patte de coin (QUAD_ORDER).
    Au début de chaque quart, les 4 pieds sont au sol et le corps se décale
    de `sway` mm à l'intérieur du triangle formé par les 3 pieds qui vont
    rester au sol ; puis la patte se lève et revient vers l'avant. Le centre
    du corps n'est donc jamais au bord du polygone d'appui. Les pattes du
    milieu ne marchent pas (Hexapod les replie).
    """

    def __init__(self, neutral_feet, period=2.8, step_height=30.0, max_stride=60.0, sway=40.0):
        self.period = period  # durée d'un cycle complet : chaque patte fait un pas
        self.step_height = step_height
        self.max_stride = max_stride
        self.sway = sway
        self.phase = 0.0
        self.shift = (0.0, 0.0)  # décalage du corps (mm) dans le plan du sol
        # Hexapod peut fournir plan(slot) -> décalage visé, calculé d'après la
        # position réelle des pieds ; sinon on vise d'après leur position neutre.
        self.plan = None
        self._target = None
        self._slot = 0
        self._shift_from = self.shift
        self._targets = {}
        for i, name in enumerate(QUAD_ORDER):
            # Le pied qui se lève est sur une diagonale ; l'autre diagonale
            # devient un bord du triangle d'appui. On s'en écarte vers le pied
            # opposé à celui qui se lève.
            partner = neutral_feet[_diagonal(name)]
            a, b = [neutral_feet[n] for n in QUAD_ORDER if n not in (name, _diagonal(name))]
            dx, dy = a[0] - b[0], a[1] - b[1]
            n = math.sqrt(dx * dx + dy * dy)
            dx, dy = dx / n, dy / n
            dot = partner[0] * dx + partner[1] * dy
            px, py = partner[0] - dot * dx, partner[1] - dot * dy
            n = math.sqrt(px * px + py * py) or 1.0
            self._targets[i] = (sway * px / n, sway * py / n)

    @property
    def swing_part(self):
        """Part du cycle pendant laquelle une patte est en l'air."""
        return (1.0 - QUAD_SHIFT) / 4

    @property
    def stance_time(self):
        return (1.0 - self.swing_part) * self.period

    @property
    def at_rest(self):
        q = self.phase * 4
        return abs(q - round(q)) < 1e-9

    def reset(self):
        self.phase = 0.0
        self._slot = 0
        self._target = None
        self.shift = self._shift_from = (0.0, 0.0)

    def advance(self, dt, moving):
        if not moving and self.at_rest:
            # Arrêté : le corps revient doucement au centre, pieds au sol.
            step = 60.0 * dt
            x, y = self.shift
            n = math.sqrt(x * x + y * y)
            k = max(0.0, n - step) / n if n else 0.0
            self.shift = self._shift_from = (x * k, y * k)
            return
        new_phase = self.phase + dt / self.period
        if not moving:
            # Arrêt au prochain quart : tous les pieds au sol.
            new_phase = min(new_phase, (int(self.phase * 4 + 1e-9) + 1) / 4.0)
        self.phase = new_phase % 1.0
        if abs(self.phase - 1.0) < 1e-9:
            self.phase = 0.0
        slot = int(self.phase * 4 + 1e-9) % 4
        if slot != self._slot or self._target is None:
            self._slot = slot
            self._shift_from = self.shift
            self._target = self.plan(slot) if self.plan else self._targets[slot]
        u = min(1.0, max(0.0, self.phase * 4 - slot))
        tx, ty = self._target
        if u < QUAD_SHIFT:
            s = u / QUAD_SHIFT
            s = s * s * (3 - 2 * s)
            fx, fy = self._shift_from
            self.shift = (fx + (tx - fx) * s, fy + (ty - fy) * s)
        else:
            self.shift = (tx, ty)

    def swing(self, leg_name):
        if leg_name not in QUAD_ORDER:
            return None  # patte repliée : gérée par Hexapod
        w = self.swing_part
        u = (self.phase - (QUAD_ORDER.index(leg_name) + QUAD_SHIFT) / 4) % 1.0
        return u / w if u < w - 1e-9 else None

    def time_to_lift(self, leg_name):
        u = (self.phase - (QUAD_ORDER.index(leg_name) + QUAD_SHIFT) / 4) % 1.0
        return (1.0 - u) * self.period

    def feet_on_ground(self):
        return [n for n in QUAD_ORDER if self.swing(n) is None or self.swing(n) == 0.0]


def _diagonal(name):
    """Patte de coin opposée en diagonale (avant gauche <-> arrière droite...)."""
    return {"LF": "RR", "RR": "LF", "RF": "LR", "LR": "RF"}[name]
