"""Machine d'états du robot et protocole texte avec le PC.

Le même code tourne sur l'ESP32 (MicroPython, lien série USB) et sur le PC en
mode simulation. Le PC n'envoie que des consignes de haut niveau ; l'IK et la
démarche sont calculées ici, sur le robot, pour que la marche reste fluide
même si le lien a des à-coups, et que le robot s'arrête seul si le lien tombe.

Commandes (une par ligne, PC -> robot) :

    PING                          -> OK PONG
    POWER 1|0                     servos alimentés (posture assise) / relâchés
    STAND                         se lever (depuis assis)
    SIT                           s'asseoir (finit le pas en cours)
    MOVE vx vy yaw                consigne de marche en mm/s, mm/s, deg/s ;
                                  à renvoyer au moins toutes les 0,5 s
    POSE x y z roll pitch yaw     décalage du corps en mm et degrés
    GAIT period step_height       durée du cycle (s) et hauteur de pas (mm)
    CAL                           pose de calibration : fémur à plat, tibia vertical (+ trim)
    TEST leg joint                fait osciller un seul servo pendant 3 s (vérif. câblage),
                                  les autres en pose de calibration ; refusé debout
    TRIM leg joint deg            règle le trim d'une articulation (ex. TRIM RF femur -3)
    SAVE                          enregistre les trims dans la mémoire du robot
    GETCAL                        -> CAL {"RF": [coxa, femur, tibia], ...}
    GEOM                          -> GEOM {...} géométrie du robot (pour la vue 3D)

Réponses (robot -> PC) : `OK ...`, `ERR ...`, `CAL {...}`, et ~20 fois par
seconde une ligne de télémétrie :

    T <mode> <phase> <18 angles en milliradians, pattes RF RM RR LF LM LR,
       coxa fémur tibia pour chacune>
"""

import json
import math
import os

from .body_ik import BodyPose
from .gait import GaitCommand
from .leg_ik import LegAngles, UnreachableError
from .robot import Hexapod

DEG = math.pi / 180.0

OFF = "off"
SIT = "sit"
STANDING = "standing"
STAND = "stand"
SITTING = "sitting"
CALIBRATION = "cal"
TEST = "test"

# Pose de calibration (angles IK en radians) : coxa dans l'axe de la fixation,
# fémur à l'horizontale, tibia vertical vers le bas.
CALIBRATION_POSE = (0.0, 0.0, -math.pi / 2)
RISE_TIME = 2.0  # s pour se lever ou s'asseoir
WATCHDOG = 0.5  # s sans MOVE -> arrêt
TELEMETRY_PERIOD = 0.05  # s
TEST_DURATION = 3.0  # s
TEST_AMPLITUDE = 20.0 * math.pi / 180.0
JOINTS = ("coxa", "femur", "tibia")


class Controller:
    def __init__(self, config, driver, calib_path="calib.json"):
        self.robot = Hexapod(config, driver)
        self.driver = driver
        self.calib_path = calib_path
        self.mode = OFF
        self.sit_drop = config.sit_drop
        self.height = -self.sit_drop  # hauteur de base du corps (0 = debout)
        self.user_pose = BodyPose()
        self.since_move = 0.0
        self.since_telemetry = TELEMETRY_PERIOD
        self.angles = self._safe_solve() or self._zero_angles()
        self.load_calibration()

    # --- boucle -----------------------------------------------------------

    def tick(self, dt):
        """À appeler à ~50 Hz. Retourne la ligne de télémétrie quand il le faut."""
        r = self.robot
        if self.mode == STAND:
            self.since_move += dt
            if self.since_move > WATCHDOG:
                r.command = GaitCommand()
        elif self.mode == STANDING:
            self.height = min(0.0, self.height + self.sit_drop / RISE_TIME * dt)
            if self.height == 0.0:
                self.mode = STAND
        elif self.mode == SITTING:
            r.command = GaitCommand()
            if r.gait.phase in (0.0, 0.5):  # tous les pieds au sol
                self.height = max(-self.sit_drop, self.height - self.sit_drop / RISE_TIME * dt)
                if self.height == -self.sit_drop:
                    self.mode = SIT

        elif self.mode == TEST:
            self.test_time += dt
            self.angles = self._calibration_angles()
            if self.test_time < TEST_DURATION:
                a = self.angles[self.test_leg]
                offset = TEST_AMPLITUDE * math.sin(2 * math.pi * self.test_time)
                setattr(a, self.test_joint, getattr(a, self.test_joint) + offset)
            else:
                self.mode = CALIBRATION
            self._send_angles()

        if self.mode in (SIT, STANDING, STAND, SITTING):
            self._apply_pose()
            try:
                self.angles = r.update(dt)
            except UnreachableError:
                # On garde la dernière position atteignable.
                r.command = GaitCommand()

        self.since_telemetry += dt
        if self.since_telemetry >= TELEMETRY_PERIOD:
            self.since_telemetry = 0.0
            return self.telemetry()
        return None

    def telemetry(self):
        values = []
        for leg in self.robot.config.legs:
            a = self.angles[leg.name]
            values += [a.coxa, a.femur, a.tibia]
        phase = self.robot.gait.phase
        return "T %s %.3f %s" % (self.mode, phase, " ".join(str(int(v * 1000)) for v in values))

    # --- commandes --------------------------------------------------------

    def handle(self, line):
        """Exécute une ligne de commande et retourne la réponse."""
        parts = line.strip().split()
        if not parts:
            return None
        name, args = parts[0].upper(), parts[1:]
        handler = getattr(self, "cmd_" + name.lower(), None)
        if handler is None:
            return "ERR commande inconnue : " + name
        try:
            return handler(*args)
        except (TypeError, ValueError) as e:
            return "ERR %s : %s" % (name, e)

    def cmd_ping(self):
        return "OK PONG"

    def cmd_power(self, on):
        if on == "1":
            if self.mode in (OFF, CALIBRATION, TEST):
                self.mode = SIT
                self.height = -self.sit_drop
                self.robot.command = GaitCommand()
                self.robot.gait.phase = 0.0
                self._apply_pose()
                self.angles = self.robot.update(0.0)
            return "OK POWER 1"
        self.mode = OFF
        self.robot.command = GaitCommand()
        if hasattr(self.driver, "relax"):
            self.driver.relax()
        return "OK POWER 0"

    def cmd_stand(self):
        if self.mode not in (SIT, SITTING, STAND):
            return "ERR STAND : alimenter le robot d'abord (POWER 1)"
        if self.mode != STAND:
            self.mode = STANDING
        return "OK STAND"

    def cmd_sit(self):
        if self.mode not in (STAND, STANDING, SIT):
            return "ERR SIT : le robot n'est pas debout"
        if self.mode != SIT:
            self.mode = SITTING
        return "OK SIT"

    def cmd_move(self, vx, vy, yaw):
        if self.mode != STAND:
            return "ERR MOVE : le robot doit être debout"
        self.robot.command = GaitCommand(float(vx), float(vy), float(yaw) * DEG)
        self.since_move = 0.0
        return None  # envoyé en continu : pas d'accusé de réception

    def cmd_pose(self, x, y, z, roll, pitch, yaw):
        self.user_pose = BodyPose(
            _clamp(float(x), 30), _clamp(float(y), 30), _clamp(float(z), 30),
            _clamp(float(roll), 20) * DEG, _clamp(float(pitch), 20) * DEG,
            _clamp(float(yaw), 25) * DEG,
        )
        return None

    def cmd_gait(self, period, step_height):
        gait = self.robot.gait
        gait.period = max(0.4, min(3.0, float(period)))
        gait.step_height = max(5.0, min(80.0, float(step_height)))
        return "OK GAIT %.2f %.0f" % (gait.period, gait.step_height)

    def cmd_cal(self):
        """Pose de calibration, choisie pour être facile à vérifier à l'œil :
        coxa perpendiculaire au corps, fémur à l'horizontale, tibia vertical.
        On fixe les palonniers dans cette pose, puis on règle les trims
        jusqu'à ce que le robot réel corresponde à la pose affichée sur le PC."""
        self.mode = CALIBRATION
        self.robot.command = GaitCommand()
        self.angles = self._calibration_angles()
        self._send_angles()
        return "OK CAL"

    def cmd_test(self, leg_name, joint_name):
        self._joint(leg_name, joint_name)  # valide les noms
        if self.mode in (STAND, STANDING, SITTING):
            return "ERR TEST : asseoir le robot d'abord"
        self.mode = TEST
        self.robot.command = GaitCommand()
        self.test_leg = leg_name.upper()
        self.test_joint = joint_name.lower()
        self.test_time = 0.0
        return "OK TEST %s %s" % (self.test_leg, self.test_joint)

    def cmd_trim(self, leg_name, joint_name, deg):
        joint = self._joint(leg_name, joint_name)
        joint.trim_deg = _clamp(float(deg), 45)
        if self.mode == CALIBRATION:
            self._send_angles()
        return "OK TRIM %s %s %.1f" % (leg_name.upper(), joint_name.lower(), joint.trim_deg)

    def cmd_save(self):
        with open(self.calib_path, "w") as f:
            f.write(json.dumps(self.calibration()))
        return "OK SAVE"

    def cmd_getcal(self):
        return "CAL " + json.dumps(self.calibration())

    def cmd_geom(self):
        return "GEOM " + json.dumps(self.robot.config.to_dict())

    # --- calibration ------------------------------------------------------

    def calibration(self):
        return {
            leg.name: [leg.coxa.trim_deg, leg.femur.trim_deg, leg.tibia.trim_deg]
            for leg in self.robot.config.legs
        }

    def load_calibration(self):
        try:
            # stat avant open : sur MicroPython (port unix), un open() qui
            # échoue peut fermer le descripteur 0 (stdin) au passage du GC.
            os.stat(self.calib_path)
            with open(self.calib_path) as f:
                data = json.loads(f.read())
        except (OSError, ValueError):
            return
        for leg in self.robot.config.legs:
            trims = data.get(leg.name)
            if trims and len(trims) == 3:
                leg.coxa.trim_deg, leg.femur.trim_deg, leg.tibia.trim_deg = trims

    # --- interne ----------------------------------------------------------

    def _joint(self, leg_name, joint_name):
        if joint_name.lower() not in JOINTS:
            raise ValueError("articulation inconnue : " + joint_name)
        try:
            leg = self.robot.config.leg(leg_name.upper())
        except KeyError:
            raise ValueError("patte inconnue : " + leg_name)
        return getattr(leg, joint_name.lower())

    def _apply_pose(self):
        p = self.user_pose
        self.robot.pose = BodyPose(p.x, p.y, self.height + p.z, p.roll, p.pitch, p.yaw)

    def _send_angles(self):
        from .servo import ik_to_servo_deg

        for leg in self.robot.config.legs:
            a = self.angles[leg.name]
            for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                self.driver.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))

    def _zero_angles(self):
        # Angles IK qui placent chaque servo au milieu de sa course.
        return {
            leg.name: LegAngles(leg.coxa.zero_deg * DEG, leg.femur.zero_deg * DEG, leg.tibia.zero_deg * DEG)
            for leg in self.robot.config.legs
        }

    def _calibration_angles(self):
        return {leg.name: LegAngles(*CALIBRATION_POSE) for leg in self.robot.config.legs}

    def _safe_solve(self):
        try:
            self._apply_pose()
            return self.robot.solve()
        except UnreachableError:
            return None


def _clamp(v, limit):
    return max(-limit, min(limit, v))
