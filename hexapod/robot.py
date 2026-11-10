"""Contrôleur haut niveau : démarche + pose du corps -> IK -> servos."""

import math

from .body_ik import BodyPose, _trig, body_to_leg_frame, foot_in_body, leg_to_body_frame, rotation_matrix
from .config import BATTERY_FOLD, MIDDLE_LEGS, STRAP_FOLD, TIBIA_FOLD_LIMITS
from .gait import (QUAD_ORDER, QUAD_TUCKED, TRIPOD_A, WAVE_ORDER, GaitCommand, QuadGait, TripodGait, WaveGait,
                   _diagonal, ground_velocity)
from .leg_ik import LegAngles, UnreachableError, leg_fk, leg_ik
from .servo import DryRunDriver

# La consigne de marche change progressivement (démarrages et arrêts doux).
ACCEL = 250.0  # mm/s²
YAW_ACCEL = 1.5  # rad/s²
# Au lâcher, le robot freine plus fort qu'il n'accélère (le pied en l'air
# vise alors sa place), et il s'arrête dès que tous les pieds sont posés à
# moins de STOP_TOL de leur place (pas de pas de plus juste pour les recentrer).
# Sol deviné au gyroscope (feel)
TOUCH_FROM = 0.5  # le pied touche peut-être à partir de la moitié de son pas
TOUCH_RATE = 0.1  # rad/s : vitesse de bascule qui trahit un contact (~6 °/s)
FEEL_SMOOTH = 0.06  # s : lissage de la vitesse de bascule
SINK_WINDOW = 0.5  # part du début de l'appui où un pied peut s'allonger (trou)
EXTEND_SPEED = 150.0  # mm/s : vitesse d'allongement d'un pied au-dessus d'un trou
GROUND_MAX = 25.0  # mm : écart de sol maximal deviné
DECEL = 600.0  # mm/s²
YAW_DECEL = 4.0  # rad/s²
STOP_TOL = 20.0  # mm
# Marche « vague » (5 pieds au sol) : toujours (WAVE 1), jamais (WAVE 0), ou
# seulement à petite vitesse (WAVE AUTO) : la tripode prend le relais quand le
# pied le plus rapide doit dépasser WAVE_SPEED ; la vague revient en dessous
# de WAVE_BACK × ce seuil (pas d'aller-retour autour du seuil).
WAVE_OFF, WAVE_ON, WAVE_AUTO = 0, 1, 2
WAVE_SPEED = 20.0  # mm/s
WAVE_SPEED_RANGE = (5.0, 60.0)
WAVE_BACK = 0.8
WAVE_ACCEL = 0.3  # part de ACCEL, DECEL... gardée en vague
WAVE_ENTER = 0.9  # tripode -> vague : à 90 % au plus du pas de la config
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
# Longueur de pas selon la place de chaque patte : dans chaque direction (vue
# de la patte), jusqu'où le pied peut aller de part et d'autre de sa place en
# gardant ses servos à REACH_MARGIN de leurs butées, sans que deux pattes
# voisines se touchent (coxa à moins de REACH_COXA du milieu). Les 6 pattes
# sont la même pièce : un seul tableau, calculé au démarrage.
REACH_DIRS = 24  # directions du tableau (tous les 15°)
REACH_MARGIN = 3.0  # ° gardés avant les butées
REACH_COXA = 33.0  # ° : deux voisines à ±33° restent à 66° (< 70°, elles se touchent)
REACH_MAX = 70.0  # mm : demi-pas maximal
# Le tableau est fait pour le corps au neutre. Avec une pose (baissé, penché,
# auto-équilibrage), on revient progressivement au demi-pas sûr de la config
# (max_stride / 2) : complètement à REACH_POSE mm ou REACH_POSE / 2 degrés.
REACH_POSE = 10.0
LIFT_MIN = 0.5  # pas pas moins hauts que la moitié de la hauteur réglée
LIFT_STEP = 0.1  # pas un peu moins haut, essayé avant la recherche complète
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
        self._legs = {leg.name: leg for leg in config.legs}
        # À 4 pattes, le corps se balance (sway) en plus du pas : pas plus
        # courts pour rester dans la course des servos (vérifié par les tests).
        self.quad = QuadGait(self.neutral, period=2 * self.tripod.period, step_height=config.step_height,
                             max_stride=min(config.max_stride, 0.36 * config.tibia_length),
                             sway=0.3 * config.tibia_length)
        self.quad.plan = self._plan_sway
        # Vague : chaque patte fait son pas à tour de rôle, cycle deux fois plus long.
        self.wave = WaveGait(period=2 * self.tripod.period, step_height=config.step_height,
                             max_stride=self.tripod.max_stride)
        self.wave_mode = WAVE_OFF
        self.wave_speed = WAVE_SPEED
        self._wave_key = None  # _wanted_gait : dernière consigne et sa vitesse
        self.six = self.tripod  # démarche à 6 pattes en cours (tripode ou vague)
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
        self._rest_key = None  # _move_feet : état immobile déjà calculé
        # Hauteur du sol sous chaque pied, devinée au gyroscope (feel) : + = plus
        # haut que prévu (caillou), - = plus bas (trou). Ajoutée au pied.
        self.ground = {n: 0.0 for n in self.neutral}
        self._ground_lift = {}  # hauteur du sol quand le pied s'est levé
        self._touched = set()  # pieds en l'air qui ont déjà touché quelque chose
        self.feel_rate = (0.0, 0.0)
        self._last_angles = None  # derniers angles valides (filet de sécurité)
        self._solve_key = None
        self._written = None  # derniers angles envoyés aux servos
        self.feel_ground = False
        self.time_scale = 1.0  # < 1 : la démarche ralentit (corps qui bascule trop vite)
        self.travel = (0.0, 0.0, 0.0)  # déplacement du corps au dernier update (mm, mm, rad, repère corps)
        # Place de chaque pied selon la pose (décalage par rapport au neutre) :
        # `home` = là où il se pose, `home_goal` = la meilleure place pour la pose actuelle.
        self._radial = {}
        for leg in config.legs:
            p = self.neutral[leg.name]
            dx, dy = p[0] - leg.mount_x, p[1] - leg.mount_y
            d = math.sqrt(dx * dx + dy * dy)
            self._radial[leg.name] = (dx / d, dy / d)
        self._reach = self._reach_table(config.legs[0])
        # Marge (°) avant les butées au neutre : on cherche à la retrouver dans toute pose.
        self._home_target = {leg.name: self._home_margin(leg, BodyPose(), 0.0) for leg in config.legs}
        self._home_wait = 0.0
        self._placed = set()  # pieds en l'air qui ont déjà pris leur nouvelle place
        self.replace_feet = True  # faux : pas de pas sur place pour replacer les pieds (en se levant…)
        self.home = {n: (0.0, 0.0) for n in self.neutral}
        self.home_goal = {n: (0.0, 0.0) for n in self.neutral}
        self._home_key = {n: (0.0,) * 6 for n in self.neutral}
        self._home_checked, self._home_checked_legs = None, None
        self._reach_cache = None

    def set_legs(self, n):
        """Passe en marche à 6 ou 4 pattes. Seulement à l'arrêt, pieds au sol."""
        if n not in (4, 6):
            raise ValueError("4 ou 6 pattes")
        if n == self.legs:
            return
        if not (self.command.is_zero and self.at_rest):
            raise ValueError("arrêter le robot d'abord")
        self.legs = n
        self.gait = self.quad if n == 4 else self.six
        # Les pieds restent où ils sont (posés, à moins de STOP_TOL de leur place).
        self.gait.reset()
        self.current = GaitCommand()
        self._lift = {}
        self._placed = set()
        self._rest_key = None

    def reset_gait(self):
        self.gait.reset()
        self.current = GaitCommand()
        self.offsets = {n: (0.0, 0.0, 0.0) for n in self.neutral}
        self._lift = {}
        self._placed = set()
        self._rest_key = None
        self.home = {n: (0.0, 0.0) for n in self.neutral}
        self.home_goal = {n: (0.0, 0.0) for n in self.neutral}
        self._home_key = {n: (0.0,) * 6 for n in self.neutral}
        self._home_checked, self._home_checked_legs = None, None
        self._reach_cache = None
        self._forget_ground()

    def _forget_ground(self):
        self.ground = {n: 0.0 for n in self.neutral}
        self._ground_lift = {}
        self._touched = set()
        self.feel_rate = (0.0, 0.0)

    def feel(self, dt, roll_rate, pitch_rate):
        """Devine au gyroscope où le sol n'est pas à la hauteur prévue.

        Un pied qui descend et touche un caillou soulève son coin du corps :
        on arrête sa descente là. Un pied qui vient de se poser au-dessus
        d'un trou laisse son coin s'enfoncer : on l'allonge jusqu'à ce que ça
        cesse. Avec 3 pieds qui bougent ensemble, on attribue la bascule au
        pied le plus du côté qui monte (ou qui descend)."""
        if not self.feel_ground:
            return
        # Lissé sur ~2 ticks : les vibrations des servos ne font pas croire à un contact.
        fr, fp = self.feel_rate
        k = min(1.0, dt / FEEL_SMOOTH) if dt else 1.0
        roll_rate, pitch_rate = fr + (roll_rate - fr) * k, fp + (pitch_rate - fp) * k
        self.feel_rate = (roll_rate, pitch_rate)
        g = self.gait
        push, sink = None, None
        for n in self.walking_legs():
            x, y = self.neutral[n][0], self.neutral[n][1]
            # > 0 : le coin de ce pied monte.
            score = (roll_rate * y - pitch_rate * x) / math.sqrt(x * x + y * y)
            s = g.swing(n)
            if s is not None and s > TOUCH_FROM and n not in self._touched:
                if score > TOUCH_RATE and (push is None or score > push[1]):
                    push = (n, score)
            elif s is None and g.time_to_lift(n) > (1.0 - SINK_WINDOW) * g.stance_time:
                if score < -TOUCH_RATE and (sink is None or score < sink[1]):
                    sink = (n, score)
        if push:
            n = push[0]
            dx, dy, dz = self.offsets[n]
            z = self.ground[n] + self.lift_scale * dz  # là où est vraiment le pied
            if z <= GROUND_MAX:
                self._touched.add(n)
                self.ground[n] = z
                self.offsets[n] = (dx, dy, 0.0)
        if sink:
            n = sink[0]
            self.ground[n] = max(-GROUND_MAX, self.ground[n] - EXTEND_SPEED * dt)

    @property
    def at_rest(self):
        """Arrêté, pieds posés à leur position neutre."""
        sx, sy = self.gait.shift  # 4 pattes : corps revenu au centre (sinon saut en passant à 6)
        return (self.current.is_zero and self.gait.at_rest and sx * sx + sy * sy < 0.25
                and self._feet_settled())

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
            n: (p[0] + self.offsets[n][0] - sx, p[1] + self.offsets[n][1] - sy,
                p[2] + lift * self.offsets[n][2] + self.ground[n])
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
        if scale == 1.0:
            return p
        return BodyPose(b.x + scale * (p.x - b.x), b.y + scale * (p.y - b.y), b.z + scale * (p.z - b.z),
                        b.roll + scale * (p.roll - b.roll), b.pitch + scale * (p.pitch - b.pitch),
                        b.yaw + scale * (p.yaw - b.yaw))

    def solve(self, pose=None, lift=1.0, check=None):
        """Angles articulaires de chaque patte pour l'état courant (pose
        demandée, ou `pose` ; pieds levés à `lift` × la hauteur de pas).
        Avec `check(leg, angles)`, retourne None dès qu'une patte ne le passe pas.

        Lève UnreachableError si une patte ne peut pas atteindre sa cible.
        """
        c = self.config
        pose = self.pose if pose is None else pose
        # foot_targets(), foot_in_body() et body_to_leg_frame() écrits ici à
        # plat, avec peu d'opérations : sur l'ESP32 chaque calcul flottant
        # crée un objet, et ce calcul est fait à chaque tick.
        (r00, r01, r02), (r10, r11, r12), (r20, r21, r22) = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
        sx, sy = self.gait.shift
        bx, by, bz = sx + pose.x, sy + pose.y, pose.z
        cl, fl, tl = c.coxa_length, c.femur_length, c.tibia_length
        neutral, offsets, ground = self.neutral, self.offsets, self.ground
        tuck = self.tuck
        angles = {}
        for leg in c.legs:
            n = leg.name
            p, o = neutral[n], offsets[n]
            dx = p[0] + o[0] - bx
            dy = p[1] + o[1] - by
            dz = p[2] + lift * o[2] + ground[n] - bz
            fx = r00 * dx + r10 * dy + r20 * dz
            fy = r01 * dx + r11 * dy + r21 * dz
            fz = r02 * dx + r12 * dy + r22 * dz
            if tuck > 0.0 and n in QUAD_TUCKED:
                t = self.tucked_foot(leg)
                k = tuck * tuck * (3 - 2 * tuck)
                fx, fy, fz = fx + (t[0] - fx) * k, fy + (t[1] - fy) * k, fz + (t[2] - fz) * k
            ca, sa = _trig(leg)
            ex, ey = fx - leg.mount_x, fy - leg.mount_y
            a = leg_ik(ca * ex + sa * ey, ca * ey - sa * ex, fz, cl, fl, tl)
            if check is not None and not check(leg, a):
                return None
            angles[n] = a
        return angles

    def update(self, dt):
        """Avance la démarche de `dt` secondes et envoie les angles aux servos."""
        goal = 1.0 if self.legs == 4 else 0.0
        step = dt / TUCK_TIME
        self.tuck = min(goal, self.tuck + step) if goal > self.tuck else max(goal, self.tuck - step)
        # Pas de marche tant que les pattes du milieu bougent.
        target = self.command if self.tuck_settled else GaitCommand()
        want = self._wanted_gait() if self.legs == 6 else self.gait
        top = 0.0
        if want is self.wave and self.gait is not want:
            # Tripode -> vague : la vague ne va pas aussi vite. Le robot
            # ralentit d'abord à ce qu'elle sait faire, puis change.
            top = WAVE_ENTER * want.max_stride / want.stance_time
            v = self._foot_speed(target)
            if v > top:
                k = top / v
                target = GaitCommand(target.vx * k, target.vy * k, target.yaw_rate * k)
        c = self.current
        # En vague, le point de pose d'un pied en l'air dépend 3 fois plus de
        # la consigne (appui 3 fois plus long) : elle change 3 fois moins vite.
        adt = dt * WAVE_ACCEL if self.gait is self.wave else dt
        self.current = GaitCommand(_approach(c.vx, target.vx, _rate(c.vx, target.vx, ACCEL, DECEL) * adt),
                                   _approach(c.vy, target.vy, _rate(c.vy, target.vy, ACCEL, DECEL) * adt),
                                   _approach(c.yaw_rate, target.yaw_rate,
                                             _rate(c.yaw_rate, target.yaw_rate, YAW_ACCEL, YAW_DECEL) * adt))
        self._home_wait -= dt
        if self._home_wait <= 0.0 and self._refresh_home():
            self._home_wait = HOME_PERIOD
        # Démarche ralentie si le corps bascule trop vite (gyroscope).
        gdt = dt * self.time_scale
        moving = not (self.current.is_zero and self._feet_settled())
        if want is not self.gait and (top == 0.0 or self._foot_speed(self.current) <= 1.05 * top):
            # Tripode <-> vague : seulement à un instant où les 6 pieds sont
            # posés. En marche, la démarche s'arrête à cet instant (comme
            # pour un arrêt), puis l'autre repart de là au tick suivant :
            # aucun pied ne saute ni ne glisse.
            if self.gait.at_rest:
                self._switch_gait(want)
            else:
                moving = False
        self.gait.advance(gdt, moving)
        self._move_feet(gdt)
        angles = self.solve_limited(dt)
        if angles is not self._written:  # mêmes angles qu'au tick d'avant : déjà envoyés
            write_servos(self.driver, self.config.legs, angles)
            self._written = angles
        return angles

    def solve_limited(self, dt=0.0):
        """Comme solve(), mais en gardant tous les servos dans leur course et
        les pièces sans contact : d'abord des pas moins hauts (jusqu'à
        LIFT_MIN), puis une pose du corps réduite. Les deux reviennent
        doucement quand c'est de nouveau possible."""
        s, l = self.pose_scale, self.lift_scale
        rs, rl = min(1.0, s + POSE_RECOVER * dt), min(1.0, l + POSE_RECOVER * dt)
        # Rien n'a bougé depuis le dernier calcul (robot immobile) : mêmes angles.
        p, b = self.pose, self.base_pose
        key = (tuple(self.offsets.values()), tuple(self.ground.values()), self.gait.shift, self.tuck,
               self.legs, p.x, p.y, p.z, p.roll, p.pitch, p.yaw, b.x, b.y, b.z, b.roll, b.pitch, b.yaw,
               s, l, rs, rl)
        if key == self._solve_key and self._last_angles is not None:
            return self._last_angles
        found = None
        tried = []
        # Avant la recherche complète (une dizaine de calculs : trop lent pour
        # l'ESP32), un pas un peu moins haut suffit le plus souvent.
        for cs, cl in ((rs, rl), (rs, l), (s, l), (s, max(LIFT_MIN, l - LIFT_STEP))):
            if (cs, cl) in tried:
                continue  # même essai qu'avant (rien à récupérer) : déjà raté
            tried.append((cs, cl))
            angles = self._try(cs, cl)
            if angles is not None:
                found = (cs, cl, angles)
                break
        if found is None:
            found = self._search(rs, rl)
        if found is None:
            # Même sans pose et sans lever les pieds, c'est trop : on garde les
            # derniers angles valides plutôt que de sauter à la pose complète.
            if self._last_angles is not None:
                return self._last_angles
            self.pose_scale = self.lift_scale = 1.0
            self.applied = self.pose
            return self.solve()
        self.pose_scale, self.lift_scale, angles = found
        self.applied = self.effective_pose(self.pose_scale)
        self._last_angles = angles
        # Mêmes entrées la prochaine fois (rien à récupérer) : on réutilise.
        self._solve_key = key if (rs, rl) == found[:2] else None
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

    # --- tripode ou vague ---------------------------------------------------

    def _wanted_gait(self):
        """Démarche à 6 pattes voulue maintenant (réglage WAVE)."""
        if self.wave_mode == WAVE_OFF:
            return self.tripod
        if self.wave_mode == WAVE_ON:
            return self.wave
        # Automatique, d'après la consigne demandée (pas celle suivie, qui
        # accélère doucement : on part tout de suite dans la bonne démarche).
        c = self.command
        if c.is_zero:
            return self.six  # arrêt : on finit dans la démarche en cours
        key = self._wave_key
        if key is None or key[0] != c.vx or key[1] != c.vy or key[2] != c.yaw_rate:
            key = self._wave_key = (c.vx, c.vy, c.yaw_rate, self._foot_speed(c))
        if self.six is self.wave:
            return self.tripod if key[3] > self.wave_speed else self.wave
        return self.wave if key[3] < WAVE_BACK * self.wave_speed else self.tripod

    def _foot_speed(self, c):
        """Vitesse (mm/s) du pied posé le plus rapide pour la consigne `c`
        (en tournant, ceux de l'extérieur)."""
        speed = 0.0
        for p in self.neutral.values():
            vx, vy = ground_velocity(p, c)
            v = math.sqrt(vx * vx + vy * vy)
            if v > speed:
                speed = v
        return speed

    def _switch_gait(self, new):
        """Change de démarche à 6 pattes, les 6 pieds posés. Les pieds restent
        où ils sont ; la nouvelle démarche commence par lever le pied (ou le
        trépied) qui a le plus reculé, celui qui a le moins de place devant lui."""
        # Le corps garde sa vitesse : la consigne suivie repart de celle que
        # l'ancienne démarche tenait vraiment (réduite à la longueur de ses pas).
        cmd = self.current = self._command_in_reach()
        first, most = None, 1e-9
        for n, p in self.neutral.items():
            vx, vy = ground_velocity(p, cmd)
            o, h = self.offsets[n], self.home[n]
            d = (o[0] - h[0]) * vx + (o[1] - h[1]) * vy
            if d > most:
                first, most = n, d
        if new is self.wave:
            new.phase = WAVE_ORDER.index(first) / 6.0 if first else 0.0
        else:
            new.phase = 0.0 if first is None or first in TRIPOD_A else 0.5
        new.shift = self.gait.shift
        self.gait = self.six = new
        # Pieds « en l'air » à 0 % de leur pas (pas encore levés) : oubliés.
        self._lift = {}
        self._ground_lift = {}
        self._placed = set()
        self._touched = set()
        self._rest_key = None

    # --- interne ----------------------------------------------------------

    def _feet_settled(self):
        # Pieds revenus à leur place (la démarche ne s'arrête que pieds posés),
        # et cette place pas trop loin de la meilleure pour la pose actuelle.
        for n in self.walking_legs():  # 4 pattes : celles du milieu sont repliées
            o, h, g = self.offsets[n], self.home[n], self.home_goal[n]
            if (o[0] - h[0]) ** 2 + (o[1] - h[1]) ** 2 > STOP_TOL * STOP_TOL:
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
        if key == self._home_checked and self.legs == self._home_checked_legs:
            return False  # rien n'a changé depuis le dernier tour complet
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
        self._home_checked, self._home_checked_legs = key, self.legs
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
        if max(out, inside) <= f0:
            return 0.0  # le neutre est déjà le mieux
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

    def _reach_table(self, leg):
        """Demi-pas possible (mm) pour chaque direction vue de la patte : le
        pied va autant d'un côté que de l'autre de sa place."""
        c = self.config
        p = self.neutral[leg.name]

        def ok(d, a):
            ux, uy = math.cos(leg.mount_angle + a), math.sin(leg.mount_angle + a)
            x, y, z = body_to_leg_frame((p[0] + d * ux, p[1] + d * uy, p[2]), leg)
            try:
                ang = leg_ik(x, y, z, c.coxa_length, c.femur_length, c.tibia_length)
            except UnreachableError:
                return False
            return leg_margin(leg, ang) >= REACH_MARGIN and abs(ang.coxa * DEG_PER_RAD) <= REACH_COXA

        def far(a):
            lo, hi = 0.0, REACH_MAX
            if ok(hi, a):
                return hi
            for _ in range(8):
                mid = (lo + hi) / 2
                if ok(mid, a):
                    lo = mid
                else:
                    hi = mid
            return lo

        dist = [far(2 * math.pi * i / REACH_DIRS) for i in range(REACH_DIRS)]
        half = REACH_DIRS // 2
        return [min(dist[i], dist[(i + half) % REACH_DIRS]) for i in range(REACH_DIRS)]

    def half_stride(self, name, vx, vy):
        """Demi-pas possible pour le pied `name` qui recule à (vx, vy)."""
        g = self.gait
        if g is self.quad:
            return g.max_stride / 2  # à 4 pattes : le balancement du corps prend de la place
        speed = math.sqrt(vx * vx + vy * vy)
        if speed < 1e-6:
            return g.max_stride / 2
        leg = self._legs[name]
        a = (math.atan2(vy, vx) - leg.mount_angle) % (2 * math.pi) * REACH_DIRS / (2 * math.pi)
        i = int(a) % REACH_DIRS
        f = a - int(a)
        t = self._reach
        base = g.max_stride / 2
        extra = max(0.0, t[i] + (t[(i + 1) % REACH_DIRS] - t[i]) * f - base)
        return base + extra * self._reach_share()

    def _reach_share(self):
        """1 au neutre, 0 dès que la pose est trop forte pour le tableau."""
        p, b = self.pose, self.base_pose
        deg = DEG_PER_RAD
        worst = max(abs(p.x - b.x), abs(p.y - b.y), abs(p.z - b.z),
                    2 * deg * abs(p.roll - b.roll), 2 * deg * abs(p.pitch - b.pitch), 2 * deg * abs(p.yaw - b.yaw))
        return max(0.0, 1.0 - worst / REACH_POSE)

    def _command_in_reach(self):
        """Consigne réduite si besoin pour qu'aucun pas ne dépasse 90 % de ce
        que permet sa patte dans sa direction (même réduction pour tous les
        pieds : pas de glissement). La marge évite qu'un pied arrive au bout
        de sa course juste avant de se lever (en tournant, ou à cause du pas
        de temps) : le corps marquerait alors une petite pause."""
        g, cmd = self.gait, self.current
        p, b = self.pose, self.base_pose
        # Même consigne, même pose, même démarche : même résultat (calcul
        # coûteux sur l'ESP32, refait seulement quand quelque chose change).
        key = (cmd.vx, cmd.vy, cmd.yaw_rate, g, g.stance_time, g.max_stride, self.legs,
               p.x - b.x, p.y - b.y, p.z - b.z, p.roll - b.roll, p.pitch - b.pitch, p.yaw - b.yaw)
        cached = self._reach_cache
        if cached is not None and cached[0] == key:
            return cached[1]
        k = 1.0
        self._halves = {}
        for n in self.walking_legs():
            vx, vy = ground_velocity(self.neutral[n], cmd)
            half = self._halves[n] = self.half_stride(n, vx, vy)
            step = math.sqrt(vx * vx + vy * vy) * g.stance_time
            if step > 1e-9:
                k = min(k, 0.9 * 2 * half / step)
        out = cmd if k >= 1.0 else GaitCommand(cmd.vx * k, cmd.vy * k, cmd.yaw_rate * k)
        self._reach_cache = (key, out)
        return out

    def _move_feet(self, dt):
        g = self.gait
        cmd = self._command_in_reach()
        rest = (g, self.legs, g.phase) if cmd.is_zero and g.at_rest else None
        if rest is not None and rest == self._rest_key:
            # Immobile, tous les pieds posés, et déjà calculé une fois ainsi :
            # rien ne bouge (calcul évité).
            self.travel = (0.0, 0.0, 0.0)
            for n in QUAD_TUCKED if self.legs == 4 else ():
                self.offsets[n] = (0.0, 0.0, 0.0)  # repliée : voir solve()
            return
        # Vitesse au sol d'un pied en (x, y) = ground_velocity() :
        # (yaw × y - vx, -vy - yaw × x), écrit à plat (calcul fait à chaque tick).
        cvx, cvy, yaw = cmd.vx, cmd.vy, cmd.yaw_rate
        st2 = g.stance_time / 2
        offsets, neutral, home = self.offsets, self.neutral, self.home
        stance = []
        swings = {}
        for n in self.walking_legs():
            dx, dy, _ = offsets[n]
            p = neutral[n]
            s = swings[n] = g.swing(n)
            # À 0 % de son pas (démarche arrêtée, tous les pieds posés, par
            # exemple en attendant de changer de démarche) : encore au sol.
            if s is None or s == 0.0:
                self._placed.discard(n)
                if self._lift.pop(n, None) is not None:
                    self._ground_lift.pop(n, None)
                    if n in self._touched:
                        self._touched.discard(n)  # déjà posé (sur un caillou) : il reste où il est
                    else:
                        # Il vient de se poser : il finit son pas sur le point visé.
                        hx, hy = home[n]
                        dx, dy = hx - (yaw * p[1] - cvx) * st2, hy + (cvy + yaw * p[0]) * st2
                        offsets[n] = (dx, dy, 0.0)
                stance.append(n)
            else:
                if n in self._touched:
                    # Posé sur ce qu'il a touché : il ne glisse pas, il recule
                    # avec le sol comme un pied d'appui.
                    self._lift.setdefault(n, (dx, dy))
                    if s > 0.0 and n not in self._placed:
                        self._placed.add(n)
                        home[n] = self.home_goal[n]
                    stance.append(n)
                    continue
                # En l'air : de là où il s'est levé jusqu'à un demi-pas en avant.
                lx, ly = self._lift.setdefault(n, (dx, dy))
                if s > 0.0 and n not in self._placed:
                    self._placed.add(n)
                    home[n] = self.home_goal[n]  # il se pose à sa meilleure place
                hx, hy = home[n]
                tx, ty = hx - (yaw * p[1] - cvx) * st2, hy + (cvy + yaw * p[0]) * st2
                k = s * s * (3 - 2 * s)
                # Il redescend vers le sol prévu (le plan des pieds).
                g0 = self._ground_lift.setdefault(n, self.ground[n])
                self.ground[n] = g0 * (1.0 - k)
                # Levée : vitesse verticale nulle au décollage et au poser (le
                # pied ne tape plus le sol), même hauteur au milieu du pas.
                u = 4.0 * s * (1.0 - s)
                z = g.step_height * u * u
                offsets[n] = (lx + (tx - lx) * k, ly + (ty - ly) * k, z)
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
            dx, dy, _ = offsets[n]
            p = neutral[n]
            x, y = p[0] + dx, p[1] + dy
            vx, vy = yaw * y - cvx, -cvy - yaw * x
            moves[n] = (vx * dt, vy * dt)
            # Pied déjà posé sur un caillou en fin de pas : il se lève au prochain cycle.
            lift = g.time_to_lift(n) if swings[n] is None else 0.0
            hx, hy = self.home[n]
            half = self._halves[n]
            if lift > dt:
                k = min(k, _fraction_inside(dx - hx, dy - hy, vx * lift, vy * lift, half))
            else:
                # Il se lève pendant ce tick : l'autre série touche le sol au
                # même moment. Le laisser finir ce dernier bout de pas au lieu
                # de figer le corps un tick (à-coup avant/arrière).
                half += math.sqrt(vx * vx + vy * vy) * dt  # pas de hypot en MicroPython
            k = min(k, _fraction_inside(dx - hx, dy - hy, vx * dt, vy * dt, half))
        for n in stance:
            dx, dy, _ = self.offsets[n]
            mx, my = moves[n]
            self.offsets[n] = (dx + mx * k, dy + my * k, 0.0)
        # Pieds posés fixes au sol : le corps a avancé de la consigne suivie × k.
        self.travel = (cmd.vx * k * dt, cmd.vy * k * dt, cmd.yaw_rate * k * dt) if stance else (0.0, 0.0, 0.0)
        for n in QUAD_TUCKED if self.legs == 4 else ():
            self.offsets[n] = (0.0, 0.0, 0.0)  # repliée : voir solve()
        self._rest_key = rest


DEG_PER_RAD = 180.0 / math.pi
_HALF_PI = math.pi / 2
_MINUS_HALF_PI = -_HALF_PI
_RAD_45, _RAD_55 = math.radians(45.0), math.radians(55.0)
_FOLD_RAD = tuple((math.radians(f), math.radians(t - FOLD_MARGIN)) for f, t in TIBIA_FOLD_LIMITS)
_BATTERY_RAD = (math.radians(BATTERY_FOLD[0]), math.radians(BATTERY_FOLD[1] - FOLD_MARGIN))
_STRAP_RAD = (math.radians(STRAP_FOLD[0]), math.radians(STRAP_FOLD[1] - FOLD_MARGIN))
AROUND = ("LF", "LM", "LR", "RR", "RM", "RF")  # sens trigo : coxa + = vers la suivante


def write_servos(driver, legs, angles):
    """Envoie les angles des pattes aux servos. Quand le pilote sait le
    faire, les voies qui se suivent sur une même carte partent en une seule
    écriture I²C (avec le câblage par défaut : une par carte)."""
    batch = getattr(driver, "set_angles", None)
    boards = {}
    for leg in legs:
        a = angles[leg.name]
        for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
            # = ik_to_servo_deg(v, joint), avec ses constantes gardées.
            k = _joint(joint)
            d = k[7] + k[8] * v
            d = k[3] if d < k[3] else k[4] if d > k[4] else d
            if batch is None:
                driver.set_angle(joint.board, joint.channel, d)
            else:
                b = boards.get(joint.board)
                if b is None:
                    b = boards[joint.board] = {}
                b[joint.channel] = d
    for board, channels in boards.items():
        order = sorted(channels)
        start = order[0]
        run = []
        for ch in order:
            if run and ch != start + len(run):
                batch(board, start, run)  # trou dans les voies : nouvelle écriture
                start, run = ch, []
            run.append(channels[ch])
        batch(board, start, run)


def leg_margin(leg, a):
    """Plus petit écart (°) entre un servo de la patte et sa butée (ou entre le
    tibia et la coxa) ; négatif s'il est dépassé."""
    # Bornes gardées (butées moins SERVO_MARGIN), écrit à plat comme leg_ok.
    c = _joint(leg.coxa)
    worst = a.coxa - c[5]
    v = c[6] - a.coxa
    if v < worst:
        worst = v
    c = _joint(leg.femur)
    v = a.femur - c[5]
    if v < worst:
        worst = v
    v = c[6] - a.femur
    if v < worst:
        worst = v
    c = _joint(leg.tibia)
    v = a.tibia - c[5]
    if v < worst:
        worst = v
    v = c[6] - a.tibia
    if v < worst:
        worst = v
    fold = _fold_limit_rad(a.femur, leg.name in MIDDLE_LEGS) + a.tibia + _HALF_PI  # moins FOLD_MARGIN
    worst = worst * DEG_PER_RAD + SERVO_MARGIN
    fold = fold * DEG_PER_RAD + FOLD_MARGIN
    return fold if fold < worst else worst


def leg_ok(leg, a):
    """Servos de la patte dans leur course (avec marge), tibia sans toucher
    la coxa, coxa sans toucher la plaque du haut."""
    # Tout en radians, bornes calculées d'avance : sur l'ESP32 chaque calcul
    # flottant crée un objet ; appelée 6 fois par tick.
    c = _joint(leg.coxa)
    v = a.coxa
    if v < c[5] or v > c[6]:
        return False
    c = _joint(leg.femur)
    femur = a.femur
    if femur < c[5] or femur > c[6]:
        return False
    c = _joint(leg.tibia)
    v = a.tibia
    if v < c[5] or v > c[6]:
        return False
    if femur <= _MINUS_HALF_PI or -v - _HALF_PI > _fold_limit_rad(femur, leg.name in MIDDLE_LEGS):
        return False
    # Fémur levé : la coxa doit moins tourner (bord de la plaque du haut).
    return femur <= _RAD_45 or abs(a.coxa) <= _RAD_55 - (femur - _RAD_45) / 3.0


def _joint(j):
    """Constantes d'un servo, gardées sur lui et recalculées si sa
    calibration change : bornes IK (rad) de leg_ok, et (a, b) tels que
    angle servo = a + b × angle IK (rad), avant les butées."""
    c = getattr(j, "_consts", None)
    if (c is None or c[0] != j.direction or c[1] != j.zero_deg or c[2] != j.trim_deg
            or c[3] != j.min_deg or c[4] != j.max_deg):
        # d = 90 + direction * (deg - zero) + trim, borné à [min, max] ± marge.
        u = (j.min_deg + SERVO_MARGIN - 90.0 - j.trim_deg) / j.direction + j.zero_deg
        w = (j.max_deg - SERVO_MARGIN - 90.0 - j.trim_deg) / j.direction + j.zero_deg
        c = j._consts = (j.direction, j.zero_deg, j.trim_deg, j.min_deg, j.max_deg,
                         (min(u, w) - 1e-9) / DEG_PER_RAD, (max(u, w) + 1e-9) / DEG_PER_RAD,
                         90.0 - j.direction * j.zero_deg + j.trim_deg, j.direction * DEG_PER_RAD)
    return c


def _fold_limit_rad(femur, middle):
    """tibia_fold_limit - FOLD_MARGIN, en radians, pour un fémur en radians."""
    pts = _FOLD_RAD
    f0, t0 = pts[0]
    if femur <= f0:
        v = t0
    else:
        v = pts[-1][1]
        for i in range(1, len(pts)):
            f1, t1 = pts[i]
            if femur <= f1:
                v = t0 + (t1 - t0) * (femur - f0) / (f1 - f0)
                break
            f0, t0 = f1, t1
    if femur <= _BATTERY_RAD[0] and v > _BATTERY_RAD[1]:
        v = _BATTERY_RAD[1]
    if middle and femur <= _STRAP_RAD[0] and v > _STRAP_RAD[1]:
        v = _STRAP_RAD[1]
    return v


def neighbours_ok(angles):
    """Deux pattes voisines ne se rapprochent pas jusqu'à se toucher."""
    for i, n in enumerate(AROUND):
        m = AROUND[(i + 1) % 6]
        if n in angles and m in angles and (angles[n].coxa - angles[m].coxa) * DEG_PER_RAD > NEIGHBOUR_LIMIT:
            return False
    return True


def angles_ok(config, angles):
    return all(leg_ok(leg, angles[leg.name]) for leg in config.legs) and neighbours_ok(angles)


def _rate(value, target, accel, decel):
    """Freinage (vers zéro, ou changement de sens) plus fort que l'accélération."""
    return accel if value * target > 0 and abs(target) > abs(value) else decel if value else accel


def _approach(value, target, step):
    if value < target:
        return min(target, value + step)
    return max(target, value - step)


def _fraction_inside(x, y, mx, my, radius):
    """Part (0..1) du déplacement (mx, my) possible depuis (x, y) sans sortir
    du disque de rayon `radius` (ou sans s'en éloigner davantage)."""
    if (x + mx) ** 2 + (y + my) ** 2 <= radius * radius:
        return 1.0
    a = mx * mx + my * my
    b = 2 * (x * mx + y * my)
    c = x * x + y * y - radius * radius
    if c >= 0:
        # Déjà dehors : il peut revenir vers le centre, pas s'en éloigner.
        return 1.0 if b <= 0 else 0.0
    return max(0.0, min(1.0, (-b + math.sqrt(b * b - 4 * a * c)) / (2 * a)))
