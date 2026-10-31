"""Générateurs de démarche.

Tripode (6 pattes) : deux trépieds alternent ; pendant que l'un pousse au sol
(appui), l'autre se lève et revient vers l'avant (transfert). Trois pieds
touchent donc toujours le sol, ce qui rend la marche statiquement stable.

4 pattes : les pattes des coins se lèvent une à une, les pattes du milieu
restent repliées (voir QuadGait).
"""

import math

TRIPOD_A = ("RF", "LM", "RR")
TRIPOD_B = ("LF", "RM", "LR")
# Marche à 4 pattes : ordre des pas (arrière gauche, avant gauche, arrière
# droite, avant droite) et pattes repliées.
QUAD_ORDER = ("LR", "LF", "RR", "RF")
QUAD_TUCKED = ("LM", "RM")


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


class TripodGait:
    def __init__(self, period=1.4, step_height=30.0, max_stride=60.0):
        self.period = period  # durée d'un cycle complet (s)
        self.step_height = step_height  # hauteur de levée du pied (mm)
        self.max_stride = max_stride  # longueur de pas maximale (mm)
        self.phase = 0.0  # 0..1

    def advance(self, dt, command):
        """Fait avancer la phase ; reste immobile si la consigne est nulle."""
        new_phase = self.phase + dt / self.period
        if command.is_zero:
            # On ne s'arrête qu'au moment où tous les pieds sont posés au
            # neutre (début ou milieu de cycle), jamais une patte en l'air.
            if self.phase in (0.0, 0.5):
                return
            rest = 0.5 if self.phase < 0.5 else 1.0
            new_phase = min(new_phase, rest)
        self.phase = new_phase % 1.0

    @property
    def at_rest(self):
        """Tous les pieds au sol, à l'arrêt possible."""
        return self.phase in (0.0, 0.5)

    def reset(self):
        self.phase = 0.0

    def stride(self, neutral_foot, command):
        """Vecteur de pas (dx, dy) d'un pied pendant une phase d'appui."""
        return _stride(neutral_foot, command, self.period / 2, self.max_stride)

    def foot_offset(self, leg_name, neutral_foot, command):
        """Décalage (dx, dy, dz) du pied par rapport à sa position neutre."""
        phase = self.phase if leg_name in TRIPOD_A else (self.phase + 0.5) % 1.0
        sx, sy = self.stride(neutral_foot, command)
        if phase < 0.5:
            # Transfert : le pied revient de l'arrière vers l'avant, en l'air.
            s = phase / 0.5
            k = s - 0.5
            dz = self.step_height * math.sin(math.pi * s)
        else:
            # Appui : le pied glisse de l'avant vers l'arrière, au sol.
            s = (phase - 0.5) / 0.5
            k = 0.5 - s
            dz = 0.0
        return sx * k, sy * k, dz

    def feet_on_ground(self):
        swinging = TRIPOD_A if 0.0 < self.phase < 0.5 else TRIPOD_B if self.phase > 0.5 else ()
        return [n for n in TRIPOD_A + TRIPOD_B if n not in swinging]


def _stride(neutral_foot, command, stance_time, max_stride):
    """Déplacement d'un pied au sol pendant `stance_time` secondes d'appui,
    pour suivre la consigne (translation et rotation autour du centre)."""
    px, py = neutral_foot[0], neutral_foot[1]
    sx = (command.vx - command.yaw_rate * py) * stance_time
    sy = (command.vy + command.yaw_rate * px) * stance_time
    n = math.sqrt(sx * sx + sy * sy)
    if n > max_stride:
        sx, sy = sx * max_stride / n, sy * max_stride / n
    return sx, sy


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
        self.phase = 0.0
        self.shift = (0.0, 0.0)  # décalage du corps (mm) dans le plan du sol
        self._slot = 0
        self._shift_from = self.shift
        self._targets = {}
        for name in QUAD_ORDER:
            # Le pied qui se lève est sur une diagonale ; l'autre diagonale
            # devient un bord du triangle d'appui. On s'en écarte vers le pied
            # opposé à celui qui se lève.
            i = QUAD_ORDER.index(name)
            partner = neutral_feet[_diagonal(name)]
            a, b = [neutral_feet[n] for n in QUAD_ORDER if n not in (name, _diagonal(name))]
            dx, dy = a[0] - b[0], a[1] - b[1]
            n = math.sqrt(dx * dx + dy * dy)
            dx, dy = dx / n, dy / n
            dot = partner[0] * dx + partner[1] * dy
            px, py = partner[0] - dot * dx, partner[1] - dot * dy
            n = math.sqrt(px * px + py * py) or 1.0
            self._targets[i] = (sway * px / n, sway * py / n)

    def reset(self):
        self.phase = 0.0
        self._slot = 0
        self.shift = self._shift_from = (0.0, 0.0)

    @property
    def swing(self):
        """Part du cycle pendant laquelle une patte est en l'air."""
        return (1.0 - QUAD_SHIFT) / 4

    @property
    def at_rest(self):
        q = self.phase * 4
        return abs(q - round(q)) < 1e-9

    def advance(self, dt, command):
        if command.is_zero and self.at_rest:
            # Arrêté : le corps revient doucement au centre.
            step = 60.0 * dt
            x, y = self.shift
            n = math.sqrt(x * x + y * y)
            k = max(0.0, n - step) / n if n else 0.0
            self.shift = self._shift_from = (x * k, y * k)
            return
        new_phase = self.phase + dt / self.period
        if command.is_zero:
            # Arrêt au prochain quart : tous les pieds au sol.
            new_phase = min(new_phase, (int(self.phase * 4 + 1e-9) + 1) / 4.0)
        self.phase = new_phase % 1.0
        if abs(self.phase - 1.0) < 1e-9:
            self.phase = 0.0
        slot = int(self.phase * 4 + 1e-9) % 4
        if slot != self._slot:
            self._slot = slot
            self._shift_from = self.shift
        u = min(1.0, max(0.0, self.phase * 4 - slot))
        tx, ty = self._targets[slot]
        if u < QUAD_SHIFT:
            s = u / QUAD_SHIFT
            s = s * s * (3 - 2 * s)
            fx, fy = self._shift_from
            self.shift = (fx + (tx - fx) * s, fy + (ty - fy) * s)
        else:
            self.shift = (tx, ty)

    def stride(self, neutral_foot, command):
        return _stride(neutral_foot, command, (1.0 - self.swing) * self.period, self.max_stride)

    def foot_offset(self, leg_name, neutral_foot, command):
        if leg_name not in QUAD_ORDER:
            return 0.0, 0.0, 0.0  # patte repliée : gérée par Hexapod
        w = self.swing
        start = (QUAD_ORDER.index(leg_name) + QUAD_SHIFT) / 4
        u = (self.phase - start) % 1.0
        sx, sy = self.stride(neutral_foot, command)
        if u < w:
            s = u / w
            k = s - 0.5
            dz = self.step_height * math.sin(math.pi * s)
        else:
            s = (u - w) / (1.0 - w)
            k = 0.5 - s
            dz = 0.0
        # Le corps avance de `shift` : vus du corps, les pieds reculent d'autant.
        return sx * k - self.shift[0], sy * k - self.shift[1], dz

    def feet_on_ground(self):
        w = self.swing
        return [n for n in QUAD_ORDER
                if (self.phase - (QUAD_ORDER.index(n) + QUAD_SHIFT) / 4) % 1.0 >= w - 1e-9]


def _diagonal(name):
    """Patte de coin opposée en diagonale (avant gauche <-> arrière droite...)."""
    return {"LF": "RR", "RR": "LF", "RF": "LR", "LR": "RF"}[name]
