"""Marche plus douce : poser du pied, démarches, décalage du corps, servos."""

import math
import unittest

from hexapod import BodyPose, GaitCommand, default_config
from hexapod.config import MIDDLE_LEGS, tibia_fold_limit
from hexapod.controller import Controller
from hexapod.gait import QUAD_TUCKED
from hexapod.robot import SAG_DEG, SAG_RATE, WAVE_AUTO, WAVE_OFF, WAVE_ON, Hexapod, angles_ok
from hexapod.servo import SLEW_DEG, DryRunDriver, soft_step
from hexapod.servo import ik_to_servo_deg

DT = 0.02
# Entre deux ticks : un pied posé suit le sol (vitesse du corps), un pied en
# l'air va au plus à ~0,5 m/s (au poser, il rejoint son point de pose, qui
# suit la consigne : quelques mm de plus pendant un freinage).
STANCE_JUMP = 4.0  # mm
SWING_JUMP = 12.0  # mm
SWAY_JUMP = 8.0  # mm : à 4 pattes, le corps se décale en plus


def walk(robot, command, seconds, watch=None):
    robot.command = command
    for _ in range(int(round(seconds / DT))):
        robot.update(DT)
        if watch:
            watch()


class Watch:
    """Vérifie à chaque tick ce qu'une marche doit toujours respecter :
    les pieds posés ne glissent pas (ils bougent tous ensemble, vus du
    corps), aucune cible de pied ne saute, aucun servo ne dépasse sa course
    et aucune pièce n'en touche une autre."""

    def __init__(self, test, robot, where=""):
        self.t, self.r, self.where = test, robot, where
        self.before = robot.foot_targets()
        self.grounded = self._grounded()
        self.gaits = [robot.gait]
        self.worst_slip = self.worst_jump = 0.0
        self.min_down = 6

    def _grounded(self):
        r = self.r
        return set(n for n in r.walking_legs() if r.offsets[n][2] == 0.0 and r.gait.swing(n) in (None, 0.0))

    def __call__(self):
        t, r = self.t, self.r
        after = r.foot_targets()
        now = self._grounded()
        self.min_down = min(self.min_down, len(now))
        both = sorted(self.grounded & now)
        for i in range(len(both)):
            for j in range(i + 1, len(both)):
                d0 = math.dist(self.before[both[i]][:2], self.before[both[j]][:2])
                d1 = math.dist(after[both[i]][:2], after[both[j]][:2])
                self.worst_slip = max(self.worst_slip, abs(d0 - d1))
                t.assertAlmostEqual(d0, d1, delta=0.05, msg=(self.where, both[i], both[j]))
        for n in r.walking_legs():
            jump = math.dist(self.before[n], after[n])
            self.worst_jump = max(self.worst_jump, jump)
            limit = SWING_JUMP if n not in both else SWAY_JUMP if r.legs == 4 else STANCE_JUMP
            t.assertLess(jump, limit, (self.where, n))
        angles = r._written
        t.assertTrue(angles_ok(r.config, angles), self.where)
        for leg in r.config.legs:
            a = angles[leg.name]
            for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                d = 90.0 + joint.direction * (math.degrees(v) - joint.zero_deg) + joint.trim_deg
                t.assertEqual(ik_to_servo_deg(v, joint), d, (self.where, leg.name))  # pas en butée
            fold = -math.degrees(a.tibia) - 90.0
            t.assertLess(fold, tibia_fold_limit(math.degrees(a.femur), leg.name in MIDDLE_LEGS), (self.where, leg.name))
        if r.gait is not self.gaits[-1]:
            self.gaits.append(r.gait)
        self.before, self.grounded = after, now


COMMANDS = ((15, 0, 0), (100, 0, 0), (-100, 0, 0), (0, 100, 0), (0, -30, 0), (70, 70, 0), (0, 0, 0.1), (0, 0, 1.0),
            (0, 0, -1.0), (40, 0, 0.3), (-60, 40, -0.5), (10, 5, 0.05), (0, 0, 0))


class WaveWalkTest(unittest.TestCase):
    def test_five_feet_down_no_slip_inside_limits(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        w = Watch(self, r, "vague")
        for c in COMMANDS:
            w.where = ("vague", c)
            walk(r, GaitCommand(*c), 6, w)
        self.assertIs(r.gait, r.wave)
        self.assertEqual(w.min_down, 5)  # jamais plus d'une patte en l'air
        self.assertTrue(r.at_rest)
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))  # sans le filet de sécurité

    def test_slower_than_tripod_but_walks(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        walk(r, GaitCommand(100.0, 0, 0), 6)
        dist = [0.0]

        def watch():
            dist[0] += r.travel[0]

        walk(r, GaitCommand(100.0, 0, 0), 10, watch)
        self.assertGreater(dist[0] / 10, 20.0)  # mm/s
        self.assertLess(dist[0] / 10, 50.0)

    def test_switch_while_walking(self):
        """Tripode <-> vague en pleine marche, à tous les moments du cycle."""
        for delay in range(0, 75, 7):
            for c in ((25, 0, 0), (0, 20, 0.1), (100, 30, -0.3), (0, 0, 0.5)):
                r = Hexapod(default_config())
                w = Watch(self, r, (delay, c))
                walk(r, GaitCommand(*c), 3 + delay * DT, w)
                r.wave_mode = WAVE_ON
                walk(r, GaitCommand(*c), 4 + delay * DT, w)
                self.assertIs(r.gait, r.wave, (delay, c))
                r.wave_mode = WAVE_OFF
                walk(r, GaitCommand(*c), 3, w)
                self.assertIs(r.gait, r.tripod, (delay, c))
                walk(r, GaitCommand(), 4, w)
                self.assertTrue(r.at_rest, (delay, c))

    def test_auto_follows_the_speed(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_AUTO
        w = Watch(self, r, "auto")
        walk(r, GaitCommand(10.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(60.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(18.0, 0, 0), 5, w)  # entre 80 % du seuil et le seuil : on ne rechange pas
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(12.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(18.0, 0, 0), 5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(0, 0, 0.5), 5, w)  # tourner vite : les pieds vont vite
        self.assertIs(r.gait, r.tripod)
        walk(r, GaitCommand(), 4, w)
        self.assertIs(r.gait, r.tripod)  # à l'arrêt : pas de changement
        self.assertTrue(r.at_rest)
        r.command = GaitCommand(5.0, 0, 0)
        r.update(DT)
        self.assertIs(r.gait, r.wave)  # départ lent : tout de suite en vague
        for c in COMMANDS:
            w.where = ("auto", c)
            walk(r, GaitCommand(*c), 5, w)
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))

    def test_default_is_tripod(self):
        r = Hexapod(default_config())
        walk(r, GaitCommand(5.0, 0, 0), 3)
        self.assertIs(r.gait, r.tripod)

    def test_four_legs_keep_their_gait(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        walk(r, GaitCommand(20.0, 0, 0), 3)
        walk(r, GaitCommand(), 4)
        r.set_legs(4)
        w = Watch(self, r, "4 pattes")
        walk(r, GaitCommand(), 1.5, w)
        walk(r, GaitCommand(50.0, 0, 0.2), 6, w)
        self.assertIs(r.gait, r.quad)
        walk(r, GaitCommand(), 6, w)
        r.set_legs(6)
        walk(r, GaitCommand(), 1.5, w)
        self.assertIs(r.gait, r.wave)
        walk(r, GaitCommand(20.0, 0, 0), 4, w)
        self.assertEqual(w.min_down, 3)  # à 4 pattes : 3 pieds posés

    def test_strong_pose_stays_inside_limits(self):
        """Le filet de sécurité (pose et levée réduites) marche aussi en vague."""
        for mode in (WAVE_ON, WAVE_AUTO):
            r = Hexapod(default_config())
            r.wave_mode = mode
            r.pose = BodyPose(20, 20, -15, math.radians(8), math.radians(8), math.radians(20))
            w = Watch(self, r, ("pose", mode))
            kept = [1.0]

            def watch():
                w()
                kept[0] = min(kept[0], r.pose_scale)

            for c in ((30, 10, 0.1), (100, 0, 0), (10, 0, 0), (0, 0, 0)):
                walk(r, GaitCommand(*c), 5, watch)
            self.assertLess(kept[0], 1.0)


class WaveShiftTest(unittest.TestCase):
    """En vague, le corps se décale à l'opposé de la patte en l'air."""

    def test_away_from_the_lifted_leg_and_smooth(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        w = Watch(self, r, "décalage")
        seen = {"max": 0.0, "step": 0.0, "dot": 0.0, "n": 0}
        last = [(0.0, 0.0)]

        def watch():
            w()
            sx, sy = r.gait.shift
            seen["max"] = max(seen["max"], math.hypot(sx, sy))
            seen["step"] = max(seen["step"], math.hypot(sx - last[0][0], sy - last[0][1]))
            last[0] = (sx, sy)
            for n in r.walking_legs():
                if r.offsets[n][2] > 0.5 * r.gait.step_height:  # patte au milieu de son pas
                    p = r.neutral[n]
                    seen["dot"] = max(seen["dot"], (sx * p[0] + sy * p[1]) / math.hypot(p[0], p[1]))
                    seen["n"] += 1

        walk(r, GaitCommand(20.0, 5.0, 0.05), 3, watch)
        seen["dot"], seen["n"] = -99.0, 0  # décalage établi : on regarde sa direction
        walk(r, GaitCommand(20.0, 5.0, 0.05), 12, watch)
        self.assertGreater(seen["n"], 100)
        self.assertLess(seen["dot"], -0.6 * r.wave_shift)  # toujours à l'opposé de la patte levée
        self.assertLessEqual(seen["max"], r.wave_shift + 1e-9)
        self.assertGreater(seen["max"], 0.95 * r.wave_shift)
        self.assertLess(seen["step"], 0.6)  # mm par tick : 30 mm/s au plus
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))
        walk(r, GaitCommand(), 4, watch)
        self.assertTrue(r.at_rest)
        self.assertEqual(r.gait.shift, (0.0, 0.0))  # à l'arrêt : corps au centre
        self.assertLess(seen["step"], 0.6)

    def test_none_in_tripod_and_standing(self):
        r = Hexapod(default_config())
        walk(r, GaitCommand(), 1)
        self.assertEqual(r.gait.shift, (0.0, 0.0))
        walk(r, GaitCommand(60.0, 0, 0), 5, lambda: self.assertEqual(r.gait.shift, (0.0, 0.0)))
        r.wave_mode = WAVE_ON  # à l'arrêt en vague : toujours rien
        walk(r, GaitCommand(), 3, lambda: self.assertEqual(r.gait.shift, (0.0, 0.0)))

    def test_zero_turns_it_off(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        r.wave_shift = 0.0
        walk(r, GaitCommand(20.0, 0, 0), 2)  # le réglage suivi part de la valeur de départ : il descend
        walk(r, GaitCommand(20.0, 0, 0), 6, lambda: self.assertEqual(r.gait.shift, (0.0, 0.0)))

    def test_fades_out_after_a_switch_to_tripod(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        r.wave_shift = 15.0
        w = Watch(self, r, "décalage fort")
        for c in COMMANDS[:6]:
            w.where = ("décalage fort", c)
            walk(r, GaitCommand(*c), 5, w)
        self.assertGreater(math.hypot(*r.gait.shift), 5.0)
        r.wave_mode = WAVE_OFF
        walk(r, GaitCommand(30.0, 0, 0), 4, w)
        self.assertIs(r.gait, r.tripod)
        self.assertEqual(r.gait.shift, (0.0, 0.0))

    def test_setting_changed_while_walking(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        w = Watch(self, r, "réglage")
        for mm in (15.0, 0.0, 10.0, 6.0):
            r.wave_shift = mm
            w.where = ("réglage", mm)
            walk(r, GaitCommand(25.0, 0, 0), 3, w)


def plain(r):
    """Angles de la géométrie seule (sans l'appoint SAG), même état."""
    keep = r._sag
    r._sag = 0.0
    try:
        return r.solve(r.effective_pose(r.pose_scale), r.lift_scale)
    finally:
        r._sag = keep


class SagTest(unittest.TestCase):
    """Les pattes qui portent poussent un peu plus (fémur baissé de SAG degrés)."""

    def test_standing_every_leg_pushes(self):
        r = Hexapod(default_config())
        self.assertEqual(r.sag, SAG_DEG)
        a = r.update(DT)
        ref = plain(r)
        for n in r.neutral:
            self.assertAlmostEqual(math.degrees(ref[n].femur - a[n].femur), SAG_DEG, places=9)
            self.assertEqual((a[n].coxa, a[n].tibia), (ref[n].coxa, ref[n].tibia))

    def check_walk(self, r, sag, command=(60.0, 20.0, 0.2), seconds=10):
        r.sag = sag
        w = Watch(self, r, ("sag", sag))
        seen = {"free": 0, "step": 0.0, "max": 0.0}
        last = {}

        def watch():
            w()
            a, ref = r._written, plain(r)
            for n in r.walking_legs():
                extra = math.degrees(ref[n].femur - a[n].femur)
                self.assertGreaterEqual(extra, -1e-9)
                self.assertLessEqual(extra, sag + 1e-9)
                z = r.offsets[n][2]
                if z == 0.0 and n not in r._touched:
                    self.assertGreater(extra, 0.75 * math.degrees(r._sag) - 1e-9)  # au sol : l'appoint (fini en 1 ou 2 ticks)
                if z > 0.9 * r.gait.step_height * r.lift_scale:
                    self.assertAlmostEqual(extra, 0.0, places=9)  # en l'air : rien
                    seen["free"] += 1
                seen["step"] = max(seen["step"], abs(extra - last.get(n, extra)))
                seen["max"] = max(seen["max"], extra)
                last[n] = extra

        walk(r, GaitCommand(), 1.5, watch)  # le réglage est rejoint en douceur
        walk(r, GaitCommand(*command), seconds, watch)
        walk(r, GaitCommand(), 4, watch)
        self.assertGreater(seen["free"], 50)
        self.assertAlmostEqual(seen["max"], sag, places=9)
        # Jamais d'à-coup : au plus SAG_RATE par seconde (plus le suivi du réglage).
        self.assertLessEqual(seen["step"], sag * SAG_RATE * DT + 5.0 * DT + 1e-9)
        self.assertTrue(r.at_rest)
        return seen

    def test_fades_around_each_step_tripod(self):
        r = Hexapod(default_config())
        self.check_walk(r, SAG_DEG)
        self.assertEqual((r.pose_scale, r.lift_scale), (1.0, 1.0))  # sans le filet de sécurité

    def test_fades_around_each_step_wave_and_four_legs(self):
        r = Hexapod(default_config())
        r.wave_mode = WAVE_ON
        self.check_walk(r, 3.0, (20.0, 5.0, 0.05), 14)
        r = Hexapod(default_config())
        r.set_legs(4)
        walk(r, GaitCommand(), 1.5)
        a, ref = r.update(DT), plain(r)
        for n in QUAD_TUCKED:
            self.assertEqual(a[n].femur, ref[n].femur)  # repliée : elle ne porte pas
        self.check_walk(r, 3.0, (40.0, 0.0, 0.1), 14)

    def test_strongest_setting_stays_inside_the_limits(self):
        for pose in (BodyPose(), BodyPose(z=-15.0, roll=0.15), BodyPose(z=20.0, pitch=-0.15, yaw=0.2)):
            r = Hexapod(default_config())
            r.pose = pose
            r.sag = 5.0
            w = Watch(self, r, ("sag fort", pose.z))  # vérifie butées et pliage à chaque tick
            for c in COMMANDS:
                walk(r, GaitCommand(*c), 3, w)

    def test_zero_turns_it_off(self):
        r = Hexapod(default_config())
        r.sag = 0.0
        walk(r, GaitCommand(), 1)

        def watch():
            a, ref = r._written, plain(r)
            for n in r.neutral:
                self.assertEqual(a[n].femur, ref[n].femur)

        walk(r, GaitCommand(60.0, 0, 0), 4, watch)

    def test_command_and_not_while_sitting(self):
        ctrl = Controller(default_config(), DryRunDriver(), "/nonexistent/calib.json")
        r = ctrl.robot
        self.assertIn(" sag 1.5", ctrl.handle("WALK"))
        self.assertIn(" sag 2.0", ctrl.handle("SAG 2"))
        self.assertIn(" sag 5.0", ctrl.handle("SAG 99"))
        self.assertIn(" sag 0.0", ctrl.handle("SAG -1"))
        self.assertTrue(ctrl.handle("SAG x").startswith("ERR"))
        ctrl.handle("SAG 2")
        ctrl.handle("POWER 1")
        for _ in range(100):
            ctrl.tick(DT)
        self.assertEqual(ctrl.mode, "sit")
        self.assertEqual(r.sag_k, 0.0)  # assis : le corps est posé, pas d'appoint
        femur = ctrl.angles["LF"].femur
        self.assertEqual(femur, plain(r)["LF"].femur)
        ctrl.handle("STAND")
        for _ in range(300):
            ctrl.tick(DT)
        self.assertEqual(ctrl.mode, "stand")
        self.assertEqual(r.sag_k, 1.0)
        self.assertAlmostEqual(math.degrees(plain(r)["LF"].femur - r._written["LF"].femur), 2.0, places=9)


def controller(slew=None):
    driver = DryRunDriver()
    ctrl = Controller(default_config(), driver, "/nonexistent/calib.json")
    if slew is not None:
        ctrl.handle("SLEW %s" % slew)
    return ctrl, driver


def ticks(ctrl, seconds, each=None):
    for _ in range(int(round(seconds / DT))):
        ctrl.tick(DT)
        if each:
            each()


class SlewTest(unittest.TestCase):
    """Sauts de consigne des servos adoucis (SLEW), marche inchangée."""

    def test_soft_step_profile(self):
        for jump in (0.3, 5.0, -30.0, 90.0):
            for accel in (300.0, SLEW_DEG, 20000.0):
                e, w, n = jump, 0.0, 0
                speeds = [0.0]
                while e != 0.0:
                    before = e
                    e, w = soft_step(e, w, accel, DT)
                    self.assertLessEqual(abs(e), abs(before))  # jamais d'éloignement
                    self.assertGreaterEqual(e * jump, 0.0)  # jamais de dépassement
                    speeds.append(abs(w))
                    n += 1
                    self.assertLess(n, 1000)
                speeds.append(0.0)
                for a, b in zip(speeds, speeds[1:]):
                    # Départ à `accel` au plus ; le freinage, calé sur les ticks, peut le dépasser un peu.
                    self.assertLessEqual(b - a, accel * DT * 1.0001 + 1e-9, (jump, accel))
                    self.assertLessEqual(a - b, accel * DT * 1.5, (jump, accel))
                ideal = 2 * math.sqrt(abs(jump) / accel)  # départ et arrivée à l'accélération maximale
                self.assertLessEqual(n * DT, ideal + 3 * DT, (jump, accel))

    def test_walking_is_sent_unchanged(self):
        """Mêmes angles envoyés avec et sans SLEW pendant toute une marche."""
        for setup in ((), ("WAVE 1",), ("LEGS 4",)):
            soft, d1 = controller()
            raw, d2 = controller(0)
            for c in (soft, raw):
                c.handle("STAND")
            ticks(soft, 6)
            ticks(raw, 6)
            for cmd in setup:
                self.assertFalse((soft.handle(cmd) or "").startswith("ERR"), cmd)
                raw.handle(cmd)
            ticks(soft, 2)
            ticks(raw, 2)
            self.assertEqual(d1.angles, d2.angles)
            for move in ("MOVE 60 0 0", "MOVE 0 30 0.3", "MOVE -40 10 -0.2", "MOVE 0 0 0", "MOVE 30 0 0"):
                for _ in range(15):
                    soft.handle(move)
                    raw.handle(move)
                    for _ in range(10):
                        soft.tick(DT)
                        raw.tick(DT)
                        self.assertEqual(d1.angles, d2.angles, (setup, move))
                        self.assertEqual(soft.telemetry(), raw.telemetry())

    def check_servos(self, ctrl, driver, seconds, accel=SLEW_DEG, speed=None):
        """Avance en vérifiant que chaque servo part en douceur (`speed` :
        vitesses à l'entrée, mises à jour). Retourne la plus grande vitesse."""
        prev = dict(driver.angles)
        speed = {k: 0.0 for k in prev} if speed is None else speed
        worst = [0.0]

        def each():
            for k, v in driver.angles.items():
                s = (v - prev[k]) / DT
                if abs(s) > abs(speed[k]):
                    self.assertLessEqual(abs(s) - abs(speed[k]), accel * DT * 1.0001 + 1e-6, k)  # accélération bornée
                worst[0] = max(worst[0], abs(s))
                speed[k], prev[k] = s, v

        ticks(ctrl, seconds, each)
        return worst[0]

    def test_jump_to_calibration_pose_is_soft(self):
        ctrl, driver = controller()
        ctrl.handle("POWER 1")
        ticks(ctrl, 2)
        self.assertEqual(ctrl.mode, "sit")
        before = dict(driver.angles)
        ctrl.handle("CAL")
        self.assertEqual(driver.angles, before)  # rien ne saute
        speed = {k: 0.0 for k in before}
        top = self.check_servos(ctrl, driver, 0.1, speed=speed)
        step = max(abs(driver.angles[k] - before[k]) for k in before)
        self.assertLess(step, 10.0)  # 0,1 s après : encore loin (sans SLEW : déjà arrivé, 60° d'un coup)
        self.assertIn("T cal", ctrl.telemetry())
        shown = [int(v) / 1000.0 for v in ctrl.telemetry().split()[3:]]
        self.assertGreater(max(abs(v - a) for v, leg in zip(shown[1::3], ctrl.robot.config.legs)
                               for a in [ctrl.angles[leg.name].femur]), 0.1)  # la télémétrie montre l'angle envoyé
        top = max(top, self.check_servos(ctrl, driver, 2, speed=speed))
        self.assertGreater(top, 50.0)
        raw, d2 = controller(0)
        raw.handle("POWER 1")
        ticks(raw, 2)
        raw.handle("CAL")
        ticks(raw, 1)
        self.assertEqual(driver.angles, d2.angles)  # arrivé exactement
        self.assertFalse(ctrl.smooth.busy)
        self.assertEqual(ctrl.telemetry(), raw.telemetry())

    def test_first_command_after_power_on_is_direct(self):
        ctrl, driver = controller()
        ctrl.handle("CAL")
        self.assertEqual(len(driver.angles), 18)  # position inconnue : envoyée telle quelle
        self.assertTrue(all(abs(v - 90.0) < 1e-9 for v in driver.angles.values()))
        ctrl.handle("POWER 0")
        driver.angles.clear()
        ctrl.handle("CAL")
        self.assertEqual(len(driver.angles), 18)

    def test_pose_is_reached_smoothly(self):
        ctrl, driver = controller()
        r = ctrl.robot
        ctrl.handle("STAND")
        ticks(ctrl, 6)
        ctrl.handle("POSE 10 -8 12 6 -5 15")
        seen = {"n": 0, "last": r.pose.z, "step": 0.0, "speed": 0.0}

        def each():
            z = r.pose.z
            if z != 12.0:
                seen["n"] += 1
            self.assertGreaterEqual(z, seen["last"])  # sans dépassement
            self.assertLessEqual(z, 12.0)
            seen["step"] = max(seen["step"], abs((z - seen["last"]) / DT - seen["speed"]))
            seen["speed"], seen["last"] = (z - seen["last"]) / DT, z

        ticks(ctrl, 0.1, each)
        self.assertLess(r.pose.z, 3.0)  # départ en douceur
        ticks(ctrl, 1.5, each)
        self.assertGreater(seen["n"], 10)  # ~0,35 s
        self.assertLess(seen["n"], 30)
        self.assertLess(seen["step"], 400.0 * DT * 1.5)  # mm/s par tick
        p = r.pose
        self.assertEqual((p.x, p.y, p.z), (10.0, -8.0, 12.0))
        self.assertAlmostEqual(math.degrees(p.roll), 6.0, places=9)
        self.assertAlmostEqual(math.degrees(p.yaw), 15.0, places=9)
        self.assertIs(ctrl.user_pose, ctrl.pose_goal)

    def test_zero_turns_everything_off(self):
        ctrl, driver = controller(0)
        ctrl.handle("STAND")
        ticks(ctrl, 6)
        ctrl.handle("POSE 0 0 12 0 0 0")
        ctrl.tick(DT)
        self.assertEqual(ctrl.robot.pose.z, 12.0)  # comme avant : d'un coup
        ctrl.handle("SIT")
        ticks(ctrl, 4)
        before = dict(driver.angles)
        ctrl.handle("CAL")
        ctrl.tick(DT)
        self.assertGreater(max(abs(driver.angles[k] - before[k]) for k in before), 20.0)
        self.assertFalse(ctrl.smooth.busy)

    def test_command(self):
        ctrl, _ = controller()
        self.assertTrue(ctrl.handle("WALK").endswith(" slew 1500"))
        self.assertTrue(ctrl.handle("SLEW 800").endswith(" slew 800"))
        self.assertTrue(ctrl.handle("SLEW -5").endswith(" slew 0"))
        self.assertTrue(ctrl.handle("SLEW 1e9").endswith(" slew 20000"))
        self.assertTrue(ctrl.handle("SLEW x").startswith("ERR"))

    def test_stand_up_and_fold_stay_inside_limits(self):
        """Se lever, replier les pattes du milieu, s'asseoir : servos toujours dans leur course."""
        ctrl, driver = controller(300)  # très doux : le plus de retard possible
        joints = {(j.board, j.channel): j for leg in ctrl.robot.config.legs for j in (leg.coxa, leg.femur, leg.tibia)}

        def each():
            for k, v in driver.angles.items():
                self.assertGreaterEqual(v, joints[k].min_deg)
                self.assertLessEqual(v, joints[k].max_deg)

        for cmd, seconds in (("STAND", 6), ("LEGS 4", 3), ("POSE 5 5 -10 8 8 20", 2), ("LEGS 6", 3), ("SIT", 5),
                             ("CAL", 2), ("TEST LF femur", 4), ("POWER 1", 3)):
            ctrl.handle(cmd)
            ticks(ctrl, seconds, each)
        self.assertFalse(ctrl.smooth.busy)


class SoftTouchdownTest(unittest.TestCase):
    def check(self, legs):
        r = Hexapod(default_config())
        if legs == 4:
            r.set_legs(4)
            walk(r, GaitCommand(), 1.5)
        h = r.gait.step_height
        prev = {}
        seen = {"peak": 0.0, "step": 0.0, "landings": 0}

        def watch():
            for n in r.walking_legs():
                z = r.offsets[n][2]
                was = prev.get(n, 0.0)
                if (z == 0.0) != (was == 0.0):
                    # Décollage ou poser : presque sans vitesse verticale.
                    seen["step"] = max(seen["step"], abs(z - was))
                    seen["landings"] += z == 0.0
                seen["peak"] = max(seen["peak"], z)
                prev[n] = z

        walk(r, GaitCommand(60.0, 20.0, 0.2), 12, watch)
        self.assertGreater(seen["landings"], 10)
        self.assertGreater(seen["peak"], 0.97 * h)  # même hauteur qu'avant au milieu du pas
        self.assertLessEqual(seen["peak"], h + 1e-9)
        # Avec sin(pi s), le dernier tick descendait de ~13 % de la hauteur (6 pattes).
        self.assertLess(seen["step"], 0.03 * h)

    def test_six_legs(self):
        self.check(6)

    def test_four_legs(self):
        self.check(4)


if __name__ == "__main__":
    unittest.main()
