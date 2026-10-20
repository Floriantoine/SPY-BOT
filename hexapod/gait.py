"""Générateur de démarche tripode.

Deux trépieds alternent : pendant que l'un pousse au sol (appui), l'autre se
lève et revient vers l'avant (transfert). Trois pieds touchent donc toujours
le sol, ce qui rend la marche statiquement stable.
"""

import math

TRIPOD_A = ("RF", "LM", "RR")
TRIPOD_B = ("LF", "RM", "LR")


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

    def stride(self, neutral_foot, command):
        """Vecteur de pas (dx, dy) d'un pied pendant une phase d'appui."""
        px, py = neutral_foot[0], neutral_foot[1]
        half = self.period / 2
        sx = (command.vx - command.yaw_rate * py) * half
        sy = (command.vy + command.yaw_rate * px) * half
        n = math.sqrt(sx * sx + sy * sy)
        if n > self.max_stride:
            sx, sy = sx * self.max_stride / n, sy * self.max_stride / n
        return sx, sy

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
