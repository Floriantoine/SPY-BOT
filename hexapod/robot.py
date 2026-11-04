"""Contrôleur haut niveau : démarche + pose du corps -> IK -> servos."""

import math

from .body_ik import BodyPose, body_to_leg_frame, foot_in_body, leg_to_body_frame
from .config import tibia_fold_limit
from .gait import QUAD_ORDER, QUAD_TUCKED, GaitCommand, QuadGait, TripodGait, _diagonal, ground_velocity
from .leg_ik import LegAngles, UnreachableError, leg_fk, leg_ik
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
# Pose du corps (POSE, auto-équilibrage) trop forte pour ce que font les
# pattes : les pas sont moins hauts, puis la pose est réduite, juste assez
# pour qu'aucun servo n'arrive en butée ni qu'une pièce en touche une autre ;
# les deux reviennent doucement.
POSE_RECOVER = 0.5  # /s : vitesse de retour à la pose / hauteur de pas demandées
SERVO_MARGIN = 1.0  # ° gardés avant chaque butée de servo
FOLD_MARGIN = 2.0  # ° gardés avant que le tibia touche la coxa
NEIGHBOUR_LIMIT = 70.0  # ° : deux pattes voisines qui se rapprochent se touchent
# Pieds replacés selon la pose : pour chaque patte, la position de pose au
# sol (le long de la patte) qui garde ses servos le plus loin de leurs
# butées, avec le corps dans la pose demandée. Le pied y va à son prochain
# pas ; à l'arrêt, le robot fait un pas sur place si un pied en est loin.
HOME_RANGE = (-30.0, 60.0)  # mm : rapproché / éloigné du corps, le long de la patte
HOME_STEP = 10.0  # mm entre deux essais
HOME_REPLACE = 8.0  # mm : à l'arrêt, un pied plus loin que ça de sa place fait un pas
HOME_PERIOD = 0.06  # s entre deux recalculs (calcul étalé, pour l'ESP32)
HOME_POSE_MM = 2.0  # la pose a changé de plus que ça (mm, ou °) : on recalcule
LIFT_MIN = 0.5  # pas pas moins hauts que la moitié de la hauteur réglée
BISECT = 4  # essais pour trouver ce qui est possible (précision 1/16)


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
        self.pose = BodyPose()  # pose demandée
        self.base_pose = BodyPose()  # partie jamais réduite (hauteur assise)
        self.pose_scale = 1.0  # part de (pose - base_pose) appliquée
        self.lift_scale = 1.0  # part de la hauteur de pas appliquée
        self.applied = BodyPose()  # pose réellement appliquée
        self.offsets = {n: (0.0, 0.0, 0.0) for n in self.neutral}  # pied - neutre (mm)
        self._lift = {}  # pieds en l'air : position (dx, dy) au moment de se lever
        self.travel = (0.0, 0.0, 0.0)  # déplacement du corps au dernier update (mm, mm, rad, repère corps)
        # Place de chaque pied selon la pose (décalage par rapport au neutre) :
        # `home` = là où il se pose, `home_goal` = la meilleure place pour la pose actuelle.
        self._radial = {}
        for leg in config.legs:
            p = self.neutral[leg.name]
            dx, dy = p[0] - leg.mount_x, p[1] - leg.mount_y
            d = math.sqrt(dx * dx + dy * dy)
            self._radial[leg.name] = (dx / d, dy / d)
        # Marge (°) avant les butées au neutre : on cherche à la retrouver dans toute pose.
        self._home_target = {leg.name: self._home_margin(leg, BodyPose(), 0.0) for leg in config.legs}
        self._home_wait = 0.0
        self._placed = set()  # pieds en l'air qui ont déjà pris leur nouvelle place
        self.replace_feet = True  # faux : pas de pas sur place pour replacer les pieds (en se levant…)
        self.home = {n: (0.0, 0.0) for n in self.neutral}
        self.home_goal = {n: (0.0, 0.0) for n in self.neutral}
        self._home_key = {n: (0.0,) * 6 for n in self.neutral}

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
        self._placed = set()
        self.home = {n: (0.0, 0.0) for n in self.neutral}
        self.home_goal = {n: (0.0, 0.0) for n in self.neutral}
        self._home_key = {n: (0.0,) * 6 for n in self.neutral}

    @property
    def at_rest(self):
        """Arrêté, pieds posés à leur position neutre."""
        return self.current.is_zero and self.gait.at_rest and self._feet_settled()

    @property
    def tuck_settled(self):
        return self.tuck == (1.0 if self.legs == 4 else 0.0)

    def walking_legs(self):
        return [n for n in self.neutral if not (self.legs == 4 and n in QUAD_TUCKED)]

    def foot_targets(self, lift=1.0):
        """Position cible de chaque pied dans le repère corps neutre
        (pieds en l'air levés à `lift` × la hauteur de pas)."""
        sx, sy = self.gait.shift  # le corps se décale : vus du corps, les pieds reculent
        return {
            n: (p[0] + self.offsets[n][0] - sx, p[1] + self.offsets[n][1] - sy, p[2] + lift * self.offsets[n][2])
            for n, p in self.neutral.items()
        }

    def tucked_foot(self, leg):
        """Pied replié, dans le repère du corps (il suit le corps, pas le sol)."""
        c = self.config
        foot = leg_fk(TUCK_ANGLES, c.coxa_length, c.femur_length, c.tibia_length)
        return leg_to_body_frame(foot, leg)

    def effective_pose(self, scale=1.0):
        """base_pose + scale × (pose - base_pose)."""
        b, p = self.base_pose, self.pose
        return BodyPose(*(getattr(b, k) + scale * (getattr(p, k) - getattr(b, k))
                          for k in ("x", "y", "z", "roll", "pitch", "yaw")))

    def solve(self, pose=None, lift=1.0, check=None):
        """Angles articulaires de chaque patte pour l'état courant (pose
        demandée, ou `pose` ; pieds levés à `lift` × la hauteur de pas).
        Avec `check(leg, angles)`, retourne None dès qu'une patte ne le passe pas.

        Lève UnreachableError si une patte ne peut pas atteindre sa cible.
        """
        c = self.config
        pose = self.pose if pose is None else pose
        targets = self.foot_targets(lift)
        angles = {}
        for leg in c.legs:
            foot = foot_in_body(targets[leg.name], pose)
            if leg.name in QUAD_TUCKED and self.tuck > 0.0:
                t = self.tucked_foot(leg)
                s = self.tuck * self.tuck * (3 - 2 * self.tuck)
                foot = tuple(a + (b - a) * s for a, b in zip(foot, t))
            x, y, z = body_to_leg_frame(foot, leg)
            a = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
            if check is not None and not check(leg, a):
                return None
            angles[leg.name] = a
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
        self._home_wait -= dt
        if self._home_wait <= 0.0 and self._refresh_home():
            self._home_wait = HOME_PERIOD
        self.gait.advance(dt, not (self.current.is_zero and self._feet_settled()))
        self._move_feet(dt)
        angles = self.solve_limited(dt)
        for leg in self.config.legs:
            a = angles[leg.name]
            for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                self.driver.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))
        return angles

    def solve_limited(self, dt=0.0):
        """Comme solve(), mais en gardant tous les servos dans leur course et
        les pièces sans contact : d'abord des pas moins hauts (jusqu'à
        LIFT_MIN), puis une pose du corps réduite. Les deux reviennent
        doucement quand c'est de nouveau possible."""
        s, l = self.pose_scale, self.lift_scale
        rs, rl = min(1.0, s + POSE_RECOVER * dt), min(1.0, l + POSE_RECOVER * dt)
        found = None
        for cs, cl in ((rs, rl), (rs, l), (s, l)):
            angles = self._try(cs, cl)
            if angles is not None:
                found = (cs, cl, angles)
                break
        if found is None:
            found = self._search(rs, rl)
        if found is None:
            # Même sans pose et sans lever les pieds, c'est trop : on garde la
            # pose demandée (les servos s'arrêtent à leur butée).
            self.pose_scale = self.lift_scale = 1.0
            self.applied = self.pose
            return self.solve()
        self.pose_scale, self.lift_scale, angles = found
        self.applied = self.effective_pose(self.pose_scale)
        return angles

    def _search(self, top_s, top_l):
        # La pose la plus forte possible avec des pas au plus bas...
        best = self._bisect(lambda v: self._try(v, LIFT_MIN), top_s)
        if best is None:
            return None
        s, angles = best
        # ... puis les pas les plus hauts possibles avec cette pose.
        best = self._bisect(lambda v: self._try(s, LIFT_MIN + (1.0 - LIFT_MIN) * v),
                            (top_l - LIFT_MIN) / (1.0 - LIFT_MIN), angles)
        v, angles = best
        return s, LIFT_MIN + (1.0 - LIFT_MIN) * v, angles

    @staticmethod
    def _bisect(attempt, top, low_angles=None):
        """Plus grande valeur de [0, top] pour laquelle attempt() réussit."""
        angles = attempt(top)
        if angles is not None:
            return top, angles
        lo, hi = 0.0, top
        angles = low_angles if low_angles is not None else attempt(0.0)
        if angles is None:
            return None
        for _ in range(BISECT):
            mid = (lo + hi) / 2
            a = attempt(mid)
            if a is None:
                hi = mid
            else:
                lo, angles = mid, a
        return lo, angles

    def _try(self, scale, lift):
        """Angles pour cette pose réduite et cette hauteur de pas, ou None si
        un servo arriverait en butée (on s'arrête à la première patte en défaut)."""
        try:
            angles = self.solve(self.effective_pose(scale), lift, leg_ok)
        except UnreachableError:
            return None
        return angles if angles is not None and neighbours_ok(angles) else None

    # --- interne ----------------------------------------------------------

    def _feet_settled(self):
        # Pieds revenus à leur place (la démarche ne s'arrête que pieds posés),
        # et cette place pas trop loin de la meilleure pour la pose actuelle.
        for n, o in self.offsets.items():
            h, g = self.home[n], self.home_goal[n]
            if abs(o[0] - h[0]) >= SETTLED or abs(o[1] - h[1]) >= SETTLED:
                return False
            if self.replace_feet and (h[0] - g[0]) ** 2 + (h[1] - g[1]) ** 2 > HOME_REPLACE * HOME_REPLACE:
                return False
        return True

    # --- place des pieds selon la pose ----------------------------------------

    def _refresh_home(self):
        """Recalcule la meilleure place d'un pied (un par appel, pour étaler
        le calcul) si la pose a changé depuis la dernière fois."""
        p, b = self.pose, self.base_pose
        # La hauteur assise (base_pose) ne compte pas : on ne déplace pas les pieds pour s'asseoir.
        key = (p.x - b.x, p.y - b.y, p.z - b.z, math.degrees(p.roll - b.roll),
               math.degrees(p.pitch - b.pitch), math.degrees(p.yaw - b.yaw))
        for leg in self.config.legs:
            n = leg.name
            if self.legs == 4 and n in QUAD_TUCKED:
                continue
            if max(abs(u - v) for u, v in zip(key, self._home_key[n])) <= HOME_POSE_MM:
                continue
            self._home_key[n] = key
            pose = BodyPose(key[0], key[1], key[2], *(math.radians(v) for v in key[3:]))
            r = self._best_radius(leg, pose)
            ux, uy = self._radial[n]
            self.home_goal[n] = (r * ux, r * uy)
            return True
        return False

    def _best_radius(self, leg, pose):
        """Décalage (mm, le long de la patte) le plus petit qui rend à la patte
        autant de marge avant les butées qu'au neutre (pied posé à un demi-pas
        autour, et levé) ; s'il n'y en a pas, celui qui en donne le plus."""
        want = self._home_target[leg.name]
        f0 = self._home_margin(leg, pose, 0.0)
        if f0 >= want:
            return 0.0
        lo, hi = HOME_RANGE
        out, inside = self._home_margin(leg, pose, HOME_STEP), self._home_margin(leg, pose, -HOME_STEP)
        sign = 1.0 if out >= inside else -1.0
        r, f = sign * HOME_STEP, max(out, inside)
        prev_r, prev_f = 0.0, f0
        while f < want:
            nr = r + sign * HOME_STEP
            if nr < lo or nr > hi:
                return r
            nf = self._home_margin(leg, pose, nr)
            if nf <= f:
                return r  # plus rien à gagner plus loin
            prev_r, prev_f, r, f = r, f, nr, nf
        # Entre les deux derniers essais : juste ce qu'il faut.
        return prev_r + (r - prev_r) * (want - prev_f) / (f - prev_f)

    def _home_margin(self, leg, pose, r):
        c = self.config
        p = self.neutral[leg.name]
        ux, uy = self._radial[leg.name]
        x0, y0 = p[0] + r * ux, p[1] + r * uy
        h = 0.35 * self.tripod.max_stride  # pied posé : un peu moins d'un demi-pas autour
        worst = 1e9
        # Posé au point, plus loin et plus près le long de la patte, et levé.
        for k, dz in ((0.0, 0.0), (h, 0.0), (-h, 0.0), (0.0, self.gait.step_height)):
            x, y, z = body_to_leg_frame(foot_in_body((x0 + k * ux, y0 + k * uy, p[2] + dz), pose), leg)
            try:
                a = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
            except UnreachableError:
                return -1e9
            worst = min(worst, leg_margin(leg, a))
        return worst

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
                self._placed.discard(n)
                if self._lift.pop(n, None) is not None:
                    # Il vient de se poser : il finit son pas sur le point visé.
                    vx, vy = ground_velocity(p, cmd)
                    hx, hy = self.home[n]
                    dx, dy = hx - vx * g.stance_time / 2, hy - vy * g.stance_time / 2
                    self.offsets[n] = (dx, dy, 0.0)
                stance.append(n)
            else:
                # En l'air : de là où il s'est levé jusqu'à un demi-pas en avant.
                lx, ly = self._lift.setdefault(n, (dx, dy))
                if s > 0.0 and n not in self._placed:
                    self._placed.add(n)
                    self.home[n] = self.home_goal[n]  # il se pose à sa meilleure place
                vx, vy = ground_velocity(p, cmd)
                hx, hy = self.home[n]
                tx, ty = hx - vx * g.stance_time / 2, hy - vy * g.stance_time / 2
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
            hx, hy = self.home[n]
            if lift > dt:
                k = min(k, _fraction_inside(dx - hx, dy - hy, vx * lift, vy * lift, half))
            k = min(k, _fraction_inside(dx - hx, dy - hy, vx * dt, vy * dt, half))
        for n in stance:
            dx, dy, _ = self.offsets[n]
            mx, my = moves[n]
            self.offsets[n] = (dx + mx * k, dy + my * k, 0.0)
        # Pieds posés fixes au sol : le corps a avancé de la consigne suivie × k.
        self.travel = (cmd.vx * k * dt, cmd.vy * k * dt, cmd.yaw_rate * k * dt) if stance else (0.0, 0.0, 0.0)
        for n in QUAD_TUCKED if self.legs == 4 else ():
            self.offsets[n] = (0.0, 0.0, 0.0)  # repliée : voir solve()


DEG_PER_RAD = 180.0 / math.pi
AROUND = ("LF", "LM", "LR", "RR", "RM", "RF")  # sens trigo : coxa + = vers la suivante


def leg_margin(leg, a):
    """Plus petit écart (°) entre un servo de la patte et sa butée (ou entre le
    tibia et la coxa) ; négatif s'il est dépassé."""
    deg = DEG_PER_RAD
    worst = 1e9
    for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
        d = 90.0 + joint.direction * (v * deg - joint.zero_deg) + joint.trim_deg
        worst = min(worst, d - joint.min_deg, joint.max_deg - d)
    femur = a.femur * deg
    return min(worst, tibia_fold_limit(femur) - (-a.tibia * deg - 90.0))


def leg_ok(leg, a):
    """Servos de la patte dans leur course (avec marge), tibia sans toucher
    la coxa, coxa sans toucher la plaque du haut."""
    deg = DEG_PER_RAD
    for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
        d = 90.0 + joint.direction * (v * deg - joint.zero_deg) + joint.trim_deg
        if d < joint.min_deg + SERVO_MARGIN or d > joint.max_deg - SERVO_MARGIN:
            return False
    femur = a.femur * deg
    if femur <= -90.0 or -a.tibia * deg - 90.0 > tibia_fold_limit(femur) - FOLD_MARGIN:
        return False
    # Fémur levé : la coxa doit moins tourner (bord de la plaque du haut).
    return femur <= 45.0 or abs(a.coxa * deg) <= 55.0 - (femur - 45.0) / 3.0


def neighbours_ok(angles):
    """Deux pattes voisines ne se rapprochent pas jusqu'à se toucher."""
    for i, n in enumerate(AROUND):
        m = AROUND[(i + 1) % 6]
        if n in angles and m in angles and (angles[n].coxa - angles[m].coxa) * DEG_PER_RAD > NEIGHBOUR_LIMIT:
            return False
    return True


def angles_ok(config, angles):
    return all(leg_ok(leg, angles[leg.name]) for leg in config.legs) and neighbours_ok(angles)


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
