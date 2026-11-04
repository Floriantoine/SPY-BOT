import math
import unittest

from hexapod.servo import PCA9685Driver
from hexapod import BodyPose, GaitCommand, Hexapod, default_config
from hexapod.body_ik import leg_to_body_frame
from hexapod.leg_ik import leg_fk
from hexapod.servo import ik_to_servo_deg
from hexapod.config import tibia_fold_limit


class HexapodTest(unittest.TestCase):
    def setUp(self):
        self.robot = Hexapod(default_config())

    def test_neutral_pose_puts_servos_near_center(self):
        self.robot.update(0.0)
        self.assertEqual(len(self.robot.driver.angles), 18)
        for deg in self.robot.driver.angles.values():
            self.assertGreater(deg, 30.0)
            self.assertLess(deg, 150.0)

    def test_walk_and_turn_stay_reachable(self):
        self.robot.command = GaitCommand(vx=80.0, vy=20.0, yaw_rate=0.3)
        self.robot.pose = BodyPose(z=10.0, roll=0.05, pitch=-0.05)
        for _ in range(200):
            self.robot.update(0.01)

    def test_feet_reach_their_targets(self):
        c = self.robot.config
        self.robot.command = GaitCommand(vx=60.0)
        self.robot.gait.phase = 0.3
        angles = self.robot.solve()
        targets = self.robot.foot_targets()
        for leg in c.legs:
            foot = leg_to_body_frame(
                leg_fk(angles[leg.name], c.coxa_length, c.femur_length, c.tibia_length), leg
            )
            for a, b in zip(foot, targets[leg.name]):
                self.assertAlmostEqual(a, b)

    def test_standby_pose_matches_reference_robot(self):
        # Posture debout de la V1.5 : fémur relevé de 28,5° (corps à 50 mm du
        # sol) et tibia vertical, donc replié de 28,5° vers le corps.
        self.robot.update(0.0)
        for leg in self.robot.config.legs:
            servos = self.robot.driver.angles
            self.assertAlmostEqual(servos[(leg.coxa.board, leg.coxa.channel)], 90.0)
            self.assertAlmostEqual(servos[(leg.femur.board, leg.femur.channel)], 61.55, places=2)
            self.assertAlmostEqual(servos[(leg.tibia.board, leg.tibia.channel)], 118.45, places=2)

    def test_eighteen_distinct_channels(self):
        c = self.robot.config
        channels = {(j.board, j.channel) for leg in c.legs for j in (leg.coxa, leg.femur, leg.tibia)}
        self.assertEqual(len(channels), 18)

    def test_scaled_dimensions_stay_reachable(self):
        robot = Hexapod(default_config({"femur": 85.2, "tibia": 178.1, "coxa": 56.0, "stand_height": 150.0}))
        robot.command = GaitCommand(vx=120.0, yaw_rate=0.5)
        for _ in range(100):
            robot.update(0.02)

    def test_servo_mapping_clamps(self):
        joint = default_config().leg("RF").coxa
        self.assertEqual(ik_to_servo_deg(math.radians(200), joint), 145.0)
        self.assertEqual(ik_to_servo_deg(0.0, joint), 90.0)


if __name__ == "__main__":
    unittest.main()


class PostureLimitsTest(unittest.TestCase):
    def test_sit_and_stand_stay_inside_servo_limits(self):
        # Un servo en butée déformerait la posture : assis comme debout, chaque
        # angle demandé doit rester dans la course du servo, avec 3° de marge.
        config = default_config()
        robot = Hexapod(config)
        for height in (0.0, -config.sit_drop):
            robot.pose = BodyPose(0, 0, height, 0, 0, 0)
            angles = robot.solve()
            for leg in config.legs:
                a = angles[leg.name]
                for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                    raw = 90.0 + joint.direction * (math.degrees(value) - joint.zero_deg)
                    self.assertGreater(raw, joint.min_deg + 3, (height, leg.name))
                    self.assertLess(raw, joint.max_deg - 3, (height, leg.name))

    def test_longest_steps_stay_inside_servo_limits(self):
        # Pas le plus long, dans toutes les directions et en tournant : les
        # pieds au sol restent dans la course des servos (un pied en l'air peut
        # toucher une butée au sommet du pas, il passe alors un peu plus bas).
        config = default_config()
        for vx, vy, yaw in ((1000, 0, 0), (0, 1000, 0), (700, 700, 0), (0, 0, 3.0)):
            robot = Hexapod(config)
            robot.command = GaitCommand(vx, vy, yaw)
            for _ in range(150):
                angles = robot.update(0.02)
                for leg in config.legs:
                    if robot.gait.swing(leg.name) is not None:
                        continue
                    a = angles[leg.name]
                    for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                        raw = 90.0 + joint.direction * (math.degrees(value) - joint.zero_deg)
                        self.assertGreater(raw, joint.min_deg, (vx, vy, yaw, leg.name))
                        self.assertLess(raw, joint.max_deg, (vx, vy, yaw, leg.name))


class MissingBoardTest(unittest.TestCase):
    """Une PCA9685 absente du bus ne doit pas empêcher le robot de démarrer."""

    def test_missing_board_is_skipped(self):
        class FakeI2C:
            def __init__(self):
                self.writes = []

            def writeto_mem(self, address, reg, data):
                if address != 0x40:
                    raise OSError(19)
                self.writes.append((address, reg))

            def readfrom_mem(self, address, reg, n):
                return bytes(n)

        import time
        if not hasattr(time, "sleep_us"):
            time.sleep_us = lambda us: None
        i2c = FakeI2C()
        driver = PCA9685Driver(i2c)
        self.assertEqual(driver.missing, [0x41])
        driver.set_angle(1, 5, 90.0)  # carte absente : ignoré
        driver.set_angle(0, 5, 90.0)
        driver.relax()
        self.assertTrue(all(a == 0x40 for a, _ in i2c.writes))


class NoSlipTest(unittest.TestCase):
    """Les pieds posés ne glissent jamais : entre deux instants, tous les pieds
    au sol bougent exactement pareil (vus du corps), même au démarrage, à
    l'arrêt, en tournant et quand la consigne change."""

    def test_feet_on_the_ground_move_together(self):
        config = default_config()
        script = [(150, 0, 0)] * 100 + [(0, 80, 0.5)] * 100 + [(-60, -60, -0.8)] * 100 + [(0, 0, 0)] * 200
        for legs in (6, 4):
            robot = Hexapod(config)
            robot.set_legs(legs)
            for _ in range(60):
                robot.update(0.02)
            before = robot.foot_targets()
            grounded = set(robot.gait.feet_on_ground())
            for vx, vy, yaw in script:
                robot.command = GaitCommand(vx, vy, yaw)
                robot.update(0.02)
                after = robot.foot_targets()
                now = set(robot.gait.feet_on_ground())
                pairs = [(before[n], after[n]) for n in grounded & now]
                if len(pairs) >= 2:
                    # Mouvement rigide : les distances entre pieds posés ne changent pas.
                    for i in range(len(pairs)):
                        for j in range(i + 1, len(pairs)):
                            d0 = math.dist(pairs[i][0][:2], pairs[j][0][:2])
                            d1 = math.dist(pairs[i][1][:2], pairs[j][1][:2])
                            self.assertAlmostEqual(d0, d1, delta=0.05)
                before, grounded = after, now
            self.assertTrue(robot.at_rest)


class CollisionTest(unittest.TestCase):
    """Le tibia ne se replie jamais jusqu'à toucher la coxa, la flasque ou la
    batterie (limites relevées dans Fusion), debout, assis, replié ou en marche."""

    # Pattes dans l'ordre du sens trigonométrique (coxa + = vers la suivante).
    AROUND = ("LF", "LM", "LR", "RR", "RM", "RF")

    def check(self, angles, where):
        for i, name in enumerate(self.AROUND):
            nxt = self.AROUND[(i + 1) % 6]
            closing = math.degrees(angles[name].coxa - angles[nxt].coxa)
            self.assertLess(closing, 70.0, (where, name, nxt))
        for name, a in angles.items():
            femur = math.degrees(a.femur)
            fold = -math.degrees(a.tibia) - 90.0  # + = pied vers le corps
            self.assertLess(fold, tibia_fold_limit(femur) - 2.0, (where, name, round(femur, 1)))
            self.assertGreater(femur, -90.0, (where, name))
            # Fémur levé : la coxa doit moins tourner (bord de la plaque du haut).
            coxa_max = 55.0 if femur <= 45 else 55.0 - (femur - 45.0) / 3.0
            self.assertLess(abs(math.degrees(a.coxa)), coxa_max, (where, name, round(femur, 1)))

    def test_no_collision(self):
        config = default_config()
        for legs in (6, 4):
            for height in (0.0, -config.sit_drop):
                robot = Hexapod(config)
                robot.set_legs(legs)
                robot.tuck = 1.0 if legs == 4 else 0.0
                robot.pose = BodyPose(0, 0, height)
                self.check(robot.solve(), (legs, height))
            for command in ((1000, 0, 0), (-1000, 0, 0), (0, 1000, 0), (700, 700, 0), (0, 0, 3.0), (0, 0, -3.0)):
                robot = Hexapod(config)
                robot.set_legs(legs)
                robot.command = GaitCommand(*command)
                for _ in range(400):
                    self.check(robot.update(0.02), (legs, command))


class SmoothWalkTest(unittest.TestCase):
    """En marche régulière, le corps avance à vitesse constante : aucun pied
    n'arrive au bout de sa course avant de se lever (sinon petites pauses)."""

    def test_no_pause(self):
        config = default_config()
        for legs in (6, 4):
            for command in ((60, 0, 0), (100, 0, 0), (0, 0, 0.52), (40, 0, 0.3), (70, 40, 0.5)):
                robot = Hexapod(config)
                robot.set_legs(legs)
                robot.command = GaitCommand(*command)
                for _ in range(300):
                    robot.update(0.02)
                speeds = []
                for _ in range(300):
                    # Position des pieds sans le balancement du corps à 4 pattes.
                    before = dict(robot.offsets)
                    grounded = [n for n in robot.walking_legs() if robot.gait.swing(n) is None]
                    robot.update(0.02)
                    after = robot.offsets
                    still = [n for n in grounded if robot.gait.swing(n) is None]
                    if not still:
                        continue  # relève des pattes
                    # Pied posé le plus éloigné du centre : sa vitesse suit celle du corps.
                    n = max(still, key=lambda m: math.hypot(*robot.neutral[m][:2]))
                    speeds.append(math.dist(before[n][:2], after[n][:2]))
                self.assertGreater(min(speeds), 0.5 * max(speeds), (legs, command))


class BodyV6FoldTest(unittest.TestCase):
    def test_middle_legs_strap_limit(self):
        from hexapod.config import tibia_fold_limit as limit
        self.assertLessEqual(limit(-53, True), 40)
        self.assertGreater(limit(-53), 40)  # autres pattes : pas de sangle
        self.assertLessEqual(limit(-73), 30)  # batterie
        self.assertEqual(limit(-30, True), limit(-30))  # postures normales inchangées
