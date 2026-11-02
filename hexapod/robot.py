"""Contrôleur haut niveau : démarche + pose du corps -> IK -> servos."""

import math

from .body_ik import BodyPose, body_to_leg_frame, foot_in_body, leg_to_body_frame
from .gait import QUAD_ORDER, QUAD_TUCKED, GaitCommand, QuadGait, TripodGait, _diagonal, ground_velocity
from .leg_ik import LegAngles, leg_fk, leg_ik
from .servo import DryRunDriver, ik_to_servo_deg

# La consigne de marche change progressivement (démarrages et arrêts doux).
ACCEL = 250.0  # mm/s²
YAW_ACCEL = 1.5  # rad/s²
SETTLED = 0.5  # mm : pied considéré revenu à sa position neutre
TUCK_TIME = 1.0  # s pour replier ou déplier les pattes du milieu
# Patte repliée : fémur relevé de 55°, tibia replié de 15° vers le corps (pied
# ~4 cm au-dessus du sol debout, ~2 cm assis), dans la course des servos et
# sans toucher la coxa.
TUCK_ANGLES = LegAngles(0.0, math.radians(55), math.radians(-105))


class Hexapod:
    """Suit la position de chaque pied par rapport à sa position neutre.

    Un pied au sol recule exactement à la vitesse du corps : il ne glisse
    jamais, même quand la consigne change. Un pied en l'air va de là où il
    s'est levé jusqu'à son point de pose, un demi-pas en avant du neutre,
    pour que le pas suivant soit centré. À l'arrêt, chaque pied revient au
    neutre à son prochain pas, puis la démarche s'arrête pieds posés.
    """

    def __init__(self, config, driver=None, gait=None):
        self.config = config
        self.driver = driver or DryRunDriver()
        self.tripod = gait or TripodGait(step_height=config.step_height, max_stride=config.max_stride)
        self.neutral = {leg.name: config.neutral_foot(leg) for leg in config.legs}
        # À 4 pattes, le corps se balance (sway) en plus du pas : pas plus
        # courts pour rester dans la course des servos (vérifié par les tests).
        self.quad = QuadGait(self.neutral, period=2 * self.tripod.period, step_height=config.step_height,
                             max_stride=min(config.max_stride, 0.36 * config.tibia_length),
                             sway=0.3 * config.tibia_length)
        self.quad.plan = self._plan_sway
        self.gait = self.tripod
        self.legs = 6  # pattes qui marchent : 6, ou 4 (pattes du milieu repliées)
        self.tuck = 0.0  # 0 = pattes du milieu au sol, 1 = repliées
        self.command = GaitCommand()  # consigne demandée
        self.current = GaitCommand()  # consigne suivie, qui rejoint la demandée en douceur
        self.pose = BodyPose()
        self.offsets = {n: (0.0, 0.0, 0.0) for n in self.neutral}  # pied - neutre (mm)
        self._lift = {}  # pieds en l'air : position (dx, dy) au moment de se lever

    def set_legs(self, n):
        """Passe en marche à 6 ou 4 pattes. Seulement à l'arrêt, pieds au sol."""
        if n not in (4, 6):
            raise ValueError("4 ou 6 pattes")
        if n == self.legs:
            return
        if not (self.command.is_zero and self.at_rest):
            raise ValueError("arrêter le robot d'abord")
        self.legs = n
        self.gait = self.quad if n == 4 else self.tripod
        self.reset_gait()

    def reset_gait(self):
        self.gait.reset()
        self.current = GaitCommand()
        self.offsets = {n: (0.0, 0.0, 0.0) for n in self.neutral}
        self._lift = {}

    @property
    def at_rest(self):
        """Arrêté, pieds posés à leur position neutre."""
        return self.current.is_zero and self.gait.at_rest and self._feet_settled()

    @property
    def tuck_settled(self):
        return self.tuck == (1.0 if self.legs == 4 else 0.0)

    def walking_legs(self):
        return [n for n in self.neutral if not (self.legs == 4 and n in QUAD_TUCKED)]

    def foot_targets(self):
        """Position cible de chaque pied dans le repère corps neutre."""
        sx, sy = self.gait.shift  # le corps se décale : vus du corps, les pieds reculent
        return {
            n: (p[0] + self.offsets[n][0] - sx, p[1] + self.offsets[n][1] - sy, p[2] + self.offsets[n][2])
            for n, p in self.neutral.items()
        }

    def tucked_foot(self, leg):
        """Pied replié, dans le repère du corps (il suit le corps, pas le sol)."""
        c = self.config
        foot = leg_fk(TUCK_ANGLES, c.coxa_length, c.femur_length, c.tibia_length)
        return leg_to_body_frame(foot, leg)

    def solve(self):
        """Angles articulaires de chaque patte pour l'état courant.

        Lève UnreachableError si une patte ne peut pas atteindre sa cible.
        """
        c = self.config
        targets = self.foot_targets()
        angles = {}
        for leg in c.legs:
            foot = foot_in_body(targets[leg.name], self.pose)
            if leg.name in QUAD_TUCKED and self.tuck > 0.0:
                t = self.tucked_foot(leg)
                s = self.tuck * self.tuck * (3 - 2 * self.tuck)
                foot = tuple(a + (b - a) * s for a, b in zip(foot, t))
            x, y, z = body_to_leg_frame(foot, leg)
            angles[leg.name] = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
        return angles

    def update(self, dt):
        """Avance la démarche de `dt` secondes et envoie les angles aux servos."""
        goal = 1.0 if self.legs == 4 else 0.0
        step = dt / TUCK_TIME
        self.tuck = min(goal, self.tuck + step) if goal > self.tuck else max(goal, self.tuck - step)
        # Pas de marche tant que les pattes du milieu bougent.
        target = self.command if self.tuck_settled else GaitCommand()
        c = self.current
        self.current = GaitCommand(_approach(c.vx, target.vx, ACCEL * dt),
                                   _approach(c.vy, target.vy, ACCEL * dt),
                                   _approach(c.yaw_rate, target.yaw_rate, YAW_ACCEL * dt))
        self.gait.advance(dt, not (self.current.is_zero and self._feet_settled()))
        self._move_feet(dt)
        angles = self.solve()
        for leg in self.config.legs:
            a = angles[leg.name]
            for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                self.driver.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))
        return angles

    # --- interne ----------------------------------------------------------

    def _feet_settled(self):
        # Pieds revenus au neutre (la démarche ne s'arrête que pieds posés).
        return all(abs(o[0]) < SETTLED and abs(o[1]) < SETTLED for o in self.offsets.values())

    def _plan_sway(self, slot):
        """Décalage du corps avant le pas de la patte n° `slot` (QUAD_ORDER) :
        à `sway` mm à l'intérieur du triangle des 3 pieds qui restent au sol,
        d'après leur position réelle au début et à la fin de ce pas."""
        g = self.gait
        lifted = QUAD_ORDER[slot]
        partner = _diagonal(lifted)
        a, b = [n for n in QUAD_ORDER if n not in (lifted, partner)]
        cmd = self._command_in_reach()
        dt = g.period / 4

        def feet(n):
            p = self.neutral[n]
            x, y = p[0] + self.offsets[n][0], p[1] + self.offsets[n][1]
            vx, vy = ground_velocity((x, y), cmd)
            return (x, y), (x + vx * dt, y + vy * dt)

        (a0, a1), (b0, b1) = feet(a), feet(b)
        ex, ey = b0[0] - a0[0], b0[1] - a0[1]
        e = math.sqrt(ex * ex + ey * ey)
        nx, ny = -ey / e, ex / e
        p = feet(partner)[0]
        if nx * (p[0] - a0[0]) + ny * (p[1] - a0[1]) < 0:
            nx, ny = -nx, -ny  # normale tournée vers le pied opposé
        need = g.sway + max(nx * q[0] + ny * q[1] for q in (a0, a1, b0, b1))
        need = max(0.0, min(g.sway, need))
        return (nx * need, ny * need)

    def _command_in_reach(self):
        """Consigne réduite si besoin pour que le pas le plus long ne dépasse
        pas 90 % de max_stride (même réduction pour tous les pieds : pas de
        glissement). La marge évite qu'un pied arrive au bout de sa course
        juste avant de se lever (en tournant, ou à cause du pas de temps) : le
        corps marquerait alors une petite pause."""
        g, cmd = self.gait, self.current
        longest = 0.0
        for n in self.walking_legs():
            vx, vy = ground_velocity(self.neutral[n], cmd)
            longest = max(longest, math.sqrt(vx * vx + vy * vy) * g.stance_time)
        reach = 0.9 * g.max_stride
        if longest <= reach:
            return cmd
        k = reach / longest
        return GaitCommand(cmd.vx * k, cmd.vy * k, cmd.yaw_rate * k)

    def _move_feet(self, dt):
        g = self.gait
        cmd = self._command_in_reach()
        half = g.max_stride / 2
        stance = []
        for n in self.walking_legs():
            dx, dy, _ = self.offsets[n]
            p = self.neutral[n]
            s = g.swing(n)
            if s is None:
                if self._lift.pop(n, None) is not None:
                    # Il vient de se poser : il finit son pas sur le point visé.
                    vx, vy = ground_velocity(p, cmd)
                    dx, dy = -vx * g.stance_time / 2, -vy * g.stance_time / 2
                    self.offsets[n] = (dx, dy, 0.0)
                stance.append(n)
            else:
                # En l'air : de là où il s'est levé jusqu'à un demi-pas en avant.
                lx, ly = self._lift.setdefault(n, (dx, dy))
                vx, vy = ground_velocity(p, cmd)
                tx, ty = -vx * g.stance_time / 2, -vy * g.stance_time / 2
                k = s * s * (3 - 2 * s)
                self.offsets[n] = (lx + (tx - lx) * k, ly + (ty - ly) * k,
                                   g.step_height * math.sin(math.pi * s))
        # Au sol, les pieds reculent à la vitesse du corps. Si l'un d'eux
        # arrive au bout de sa course (accélération en plein pas), le corps
        # ralentit un instant : même réduction pour tous, aucun pied ne glisse.
        # Au démarrage, un pied qui ne se lèvera que dans longtemps (le
        # dernier de la série à 4 pattes) n'aurait pas la place de reculer
        # jusque-là : le corps avance alors juste assez lentement pour qu'il
        # arrive au bout de sa course au moment de se lever, au lieu de
        # s'arrêter net quand il y arrive.
        moves = {}
        k = 1.0
        for n in stance:
            dx, dy, _ = self.offsets[n]
            p = self.neutral[n]
            vx, vy = ground_velocity((p[0] + dx, p[1] + dy), cmd)
            moves[n] = (vx * dt, vy * dt)
            lift = g.time_to_lift(n)
            if lift > dt:
                k = min(k, _fraction_inside(dx, dy, vx * lift, vy * lift, half))
            k = min(k, _fraction_inside(dx, dy, vx * dt, vy * dt, half))
        for n in stance:
            dx, dy, _ = self.offsets[n]
            mx, my = moves[n]
            self.offsets[n] = (dx + mx * k, dy + my * k, 0.0)
        for n in QUAD_TUCKED if self.legs == 4 else ():
            self.offsets[n] = (0.0, 0.0, 0.0)  # repliée : voir solve()


def _approach(value, target, step):
    if value < target:
        return min(target, value + step)
    return max(target, value - step)


def _fraction_inside(x, y, mx, my, radius):
    """Part (0..1) du déplacement (mx, my) possible depuis (x, y) sans sortir
    du disque de rayon `radius` (ou sans s'en éloigner davantage)."""
    if (x + mx) ** 2 + (y + my) ** 2 <= radius * radius:
        return 1.0
    if x * mx + y * my <= 0:
        return 1.0  # déjà dehors mais revient vers le centre
    a = mx * mx + my * my
    b = 2 * (x * mx + y * my)
    c = x * x + y * y - radius * radius
    if c >= 0:
        return 0.0
    return max(0.0, min(1.0, (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)))
