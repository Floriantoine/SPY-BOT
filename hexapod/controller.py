"""Machine d'états du robot et protocole texte avec le PC.

Le même code tourne sur l'ESP32 (MicroPython, lien série USB) et sur le PC en
mode simulation. Le PC n'envoie que des consignes de haut niveau ; l'IK et la
démarche sont calculées ici, sur le robot, pour que la marche reste fluide
même si le lien a des à-coups, et que le robot s'arrête seul si le lien tombe.

Commandes (une par ligne, PC -> robot) :

    PING                          -> OK PONG
    POWER 1|0                     servos alimentés (démarrage en douceur vers la
                                  posture assise) / relâchés
    STAND                         se lever (alimente d'abord les servos si besoin)
    SIT                           s'asseoir (finit le pas en cours)
    MOVE vx vy yaw                consigne de marche en mm/s, mm/s, deg/s ;
                                  à renvoyer au moins toutes les 1 s
    POSE x y z roll pitch yaw     décalage du corps en mm et degrés
    GAIT period step_height       durée du cycle (s) et hauteur de pas (mm)
    LEGS 6|4                      marche à 6 pattes, ou à 4 (pattes du milieu
                                  repliées) ; seulement à l'arrêt
    WAVE 0|1|AUTO [mm/s]          démarche à 6 pattes : tripode (0, au départ),
                                  vague (1 : une patte en l'air à la fois, 5 pieds
                                  au sol, lente), ou automatique : vague tant que
                                  le pied le plus rapide reste sous le seuil
                                  (20 mm/s au départ, 5 à 60), tripode au-dessus.
                                  En marche, le changement attend que les 6 pieds
                                  soient posés. -> WALK ...
    SHIFT mm                      en vague, le corps se décale de tant de mm à
                                  l'opposé de la patte en l'air (6 au départ, 0 à
                                  15, 0 = aucun) ; nul à l'arrêt -> WALK ...
    SAG deg                       les pattes qui portent poussent un peu plus,
                                  pour compenser la flexion sous le poids : fémur
                                  baissé de tant de degrés (1.5 au départ, 0 à 5,
                                  0 = rien), fondu au lever et au poser du pied
                                  -> WALK ...
    HOLD leg joint deg            en pose de calibration, tient ce servo à deg
                                  degrés de cette pose (±60, borné à ses butées)
                                  pour mesurer sa course au rapporteur
                                  -> OK HOLD leg joint <angle tenu>
    TRAVEL leg joint us           course réelle d'un servo : us µs d'impulsion par
    TRAVEL ALL us                 degré (10 au départ, 8 à 13). Mesure : HOLD à
    TRAVEL                        -45 puis +45, angle réel m entre les deux ->
                                  us = réglage en cours x 90 / m. Gardé par SAVE.
                                  Sans argument -> TRAVEL {json: 3 valeurs par patte}
    SLEW a                        sauts de consigne adoucis : un servo qui doit
                                  rejoindre une cible éloignée (pose de
                                  calibration, test, pattes repliées, réglage...)
                                  n'accélère pas plus que a °/s² (1500 au départ,
                                  0 à 20000, 0 = aucun adoucissement) ; POSE est
                                  rejointe en douceur de la même façon. Les
                                  consignes de la marche passent telles quelles
                                  -> WALK ...
    WALK                          -> WALK wave 0|1|auto <seuil> gait tripod|wave
                                  shift <mm> sag <deg> slew <a> : réglages de la marche et démarche à
                                  6 pattes en cours (ligne envoyée aussi quand
                                  elle change)
    CAL                           pose de calibration : fémur à plat, tibia vertical (+ trim)
    TEST leg joint                fait osciller un seul servo pendant 3 s (vérif. câblage),
                                  les autres en pose de calibration ; refusé debout
    TRIM leg joint deg            règle le trim d'une articulation (ex. TRIM RF femur -3)
    INVERT leg joint 1|0          inverse le sens d'un servo monté à l'envers
    SAVE                          enregistre les trims et sens dans la mémoire du robot
    GETCAL                        -> CAL {"RF": [coxa, femur, tibia], ...,
                                          "invert": {"RF": [0, 1, 0], ...}}
    GEOM                          -> GEOM {...} géométrie du robot (pour la vue 3D)
    SENSOR                        -> SENSOR present 0|1 on 0|1 stop <mm> : capteur avant
    SENSOR 1|0                    balayage et arrêt automatique activés / coupés
    SENSOR STOP mm                distance d'arrêt devant un obstacle
    OBST                          (simulation) -> OBST x y r x y r ... obstacles virtuels
    OBST ADD x y r | OBST CLEAR   (simulation) ajoute un cylindre / efface tout
    GOTO x y [vitesse]            aller seul au point (x, y) du sol (repère de P),
                                  en contournant les obstacles vus par le capteur
    FOLLOW [vitesse]              suivre ce qui est devant le capteur (à ~40 cm)
                                  GOTO et FOLLOW s'arrêtent (NAV off) si aucune
                                  ligne n'arrive du PC pendant 5 s : envoyer au
                                  moins un PING de temps en temps
    NAV [STOP]                    -> NAV off|follow|goto x y ; STOP reprend la main
                                  (un MOVE aussi)
    PERSON x y                    (simulation) la personne virtuelle marche jusque-là
    IMU                           -> IMU present 0|1 level 0|1 feel 0|1 ready 0|1
                                  axes +x+y+z zero <roulis> <tangage> : gyroscope
                                  MPU6050 (ready : biais mesuré, assiette connue ;
                                  zero en 0,1°)
    IMU AXES +y-x+z               orientation de la carte du gyroscope : pour l'avant,
                                  la gauche et le haut du robot, l'axe de la carte
                                  (x, y, z) qui pointe par là et son signe ; refusé
                                  si ce n'est pas une rotation ; gardé en mémoire
    IMU ZERO                      robot posé à plat : l'assiette lue devient le zéro
                                  (gardé en mémoire) ; IMU ZERO 0 l'efface
    IMU CAL                       mesure de nouveau le biais du gyroscope (1 s, robot
                                  immobile ; recommence tant qu'on le bouge)
    LEVEL 1|0                     auto-équilibrage : le corps reste horizontal
                                  (à l'assiette de POSE) même sur une pente
    FEEL 1|0                      sol deviné au gyroscope : un pied qui tape un
                                  caillou s'arrête, un pied au-dessus d'un trou
                                  s'allonge, la marche ralentit si ça bascule vite
                                  -> IMU present ... (comme IMU)
                                  -> TILT 1|0 (spontané) : corps penché de plus de
                                  20° (arrêt de sécurité) / de nouveau droit
    TILT roll pitch               (simulation) pente du sol en degrés
    TERRAIN [1|0]                 (simulation) terrain accidenté (bosses, cailloux,
                                  trous) : le robot repose sur ses pieds posés
                                  -> TERRAIN 0|1, puis TERRAINMAP (description)
                                  -> LIMIT pose 1|0 (spontané) : pose du corps
                                  réduite pour garder les servos dans leur course

Réponses (robot -> PC) : `OK ...`, `ERR ...`, `CAL {...}`, et ~20 fois par
seconde une ligne de télémétrie :

    T <mode> <phase> <18 angles en milliradians, pattes RF RM RR LF LM LR,
       coxa fémur tibia pour chacune>
    M <v réelle> <v demandée> <marge> <pose> <levée> <tick>
                                  mesures : vitesses en mm/s, plus petite marge
                                  d'un servo à sa butée en 0,1°, pose et levée
                                  gardées en %, plus long tick de la boucle en µs

Avec le capteur avant activé, en plus :

    S <angle> <distance>          une mesure : angle du balayage (deg, + à
                                  gauche), distance en mm (0 = rien en vue)
    OBSTACLE 1|0                  la marche demandée est bloquée / libre
    P <x> <y> <cap>               position estimée du robot sur le sol (mm, mrad)
    NAV arrived | NAV stuck       but atteint / impossible à atteindre (puis NAV off)
    PERSON <x> <y>                (simulation) position de la personne virtuelle
    I <roulis> <tangage> <corr. roulis> <corr. tangage>
                                  assiette mesurée et correction, en 0,1°
    B <roulis> <tangage> <sol roulis> <sol tangage> <posés>
                                  assiette du corps par rapport aux pieds et
                                  pente du sol (simulation), en 0,1° ; pieds
                                  posés (bit i = i-ème patte de la config)
    LIMIT pose 1|0                pose du corps réduite / de nouveau complète
    G <a> <b> <c> <pied> <dz>     (simulation, terrain) plan des pieds posés :
                                  z = a + b·x + c·y (mm ; b, c × 10 000), et un
                                  pied posé qui touche (indice) avec sa hauteur
                                  par rapport au pied normal (0,1 mm)
"""

import json
import math
import os

from .body_ik import BodyPose
from .gait import GaitCommand
from .config import US_PER_DEG_RANGE
from .leg_ik import LegAngles, UnreachableError
from .robot import SAG_MAX, WAVE_AUTO, WAVE_SHIFT_MAX, WAVE_SPEED_RANGE, Hexapod, _joint, leg_margin

try:  # MicroPython
    from time import ticks_diff, ticks_us
except ImportError:  # PC
    from time import perf_counter

    def ticks_us():
        return int(perf_counter() * 1e6)

    def ticks_diff(a, b):
        return a - b
from .imu import IMU_AXES, ZERO_MAX, Leveler, format_axes, parse_axes
from .nav import Navigator
from .sensor import STOP_RANGE, Scanner
from .servo import DEG_PER_RAD, SLEW_DEG, SLEW_MAX, SmoothDriver, soft_step

DEG = math.pi / 180.0

OFF = "off"
WAKE = "wake"  # démarrage en douceur vers la posture assise
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
# Démarrage : les pattes s'activent l'une après l'autre (moins d'appel de courant),
# et rejoignent la posture assise en douceur quand leur position est connue.
WAKE_STAGGER = 0.25  # s entre deux pattes dont la position est inconnue
WAKE_BLEND = 1.0  # s pour rejoindre la posture assise depuis une position connue
WATCHDOG = 1.0  # s sans MOVE -> arrêt (assez long pour ne pas couper la marche si une commande arrive en retard)
# GOTO et FOLLOW marchent sans MOVE : ils s'arrêtent si plus aucune ligne
# n'arrive du PC pendant ce temps (l'appli envoie un PING toutes les 2 s).
LINK_TIMEOUT = 5.0  # s
TELEMETRY_PERIOD = 0.05  # s
_T_FORMAT = "T %s %.3f" + " %d" * 18
TILT_STOP = 20 * math.pi / 180  # rad : au-delà, arrêt de sécurité
TILT_OK = 12 * math.pi / 180  # rad : il repart quand il est revenu en dessous
# Démarche ralentie quand le corps bascule vite (avec FEEL)
SLOW_RATE = 0.3  # rad/s : en dessous, vitesse normale
SLOW_SPAN = 0.6  # rad/s : de SLOW_RATE à SLOW_RATE + SLOW_SPAN, de 100 % à SLOW_MIN
SLOW_MIN = 0.3
SLOW_RECOVER = 1.5  # 1/s : retour à la vitesse normale
TEST_DURATION = 3.0  # s
TEST_AMPLITUDE = 20.0 * math.pi / 180.0
JOINTS = ("coxa", "femur", "tibia")
# POSE : la pose demandée est rejointe en douceur (départ et arrivée sans
# à-coup), les pieds posés ne bougent pas. Accélérations pour le réglage
# SLEW de départ ; elles suivent ce réglage (SLEW 0 : pose appliquée d'un coup).
POSE_SLEW_MM = 400.0  # mm/s² ; 15 mm en 0,4 s
POSE_SLEW_RAD = 4.0  # rad/s² ; 8° en 0,4 s
POSE_AXES = ("x", "y", "z", "roll", "pitch", "yaw")


class Controller:
    def __init__(self, config, driver, calib_path="calib.json", ranger=None, imu=None):
        self.smooth = SmoothDriver(driver)  # sauts de consigne adoucis (SLEW)
        self.robot = Hexapod(config, self.smooth)
        self._walk_gait = self.robot.six  # démarche à 6 pattes de la dernière ligne WALK
        # Capteur avant : `ranger` = VL53L1X, SimRanger (simulation) ou None (absent).
        self.scanner = Scanner(ranger, driver)
        self.wanted = GaitCommand()  # consigne MOVE, avant le filtre d'obstacle
        self.odom = (0.0, 0.0, 0.0)  # position estimée du robot sur le sol (mm, mm, rad)
        self._pending = []  # lignes à envoyer avec la prochaine télémétrie
        self.nav = Navigator()  # aller à un point / suivre (GOTO, FOLLOW)
        # Gyroscope : `imu` = MPU6050, SimIMU (simulation) ou None (absent).
        self.imu = imu
        self.leveler = Leveler()
        self.driver = driver
        self.calib_path = calib_path
        self.mode = OFF
        self.sit_drop = config.sit_drop
        self.height = -self.sit_drop  # hauteur de base du corps (0 = debout)
        self.user_pose = BodyPose()  # pose suivie : rejoint pose_goal en douceur
        self.pose_goal = self.user_pose  # pose demandée (POSE)
        self._pose_vel = [0.0] * 6
        self.pose_limited = False  # pose réduite pour rester dans la course des servos
        self._stance_names, self._stance_z = [], []
        self.tilted = False  # corps trop penché : arrêt de sécurité
        self._upright_time = 0.0
        self._free_time = 0.0
        self._went = 0.0  # mm parcourus depuis la dernière ligne M
        self._tick_us = 0  # plus long tick depuis la dernière ligne M
        self._margin_of, self._margin = None, 0.0
        self.since_move = 0.0
        self.since_line = 0.0  # s depuis la dernière ligne reçue du PC
        # Télémétrie : 20 fois par seconde ; l'ESP32 (main.py) en envoie moins,
        # chaque envoi lui coûte ~10 ms de calcul.
        self.telemetry_period = TELEMETRY_PERIOD
        self.since_telemetry = TELEMETRY_PERIOD
        self.angles = self._safe_solve() or self._zero_angles()
        self.load_calibration()

    # --- boucle -----------------------------------------------------------

    def tick(self, dt):
        """À appeler à ~50 Hz. Retourne la ligne de télémétrie quand il le faut."""
        t0 = ticks_us()
        r = self.robot
        lines, self._pending = self._pending, []
        self.since_line += dt
        if self.user_pose is not self.pose_goal:
            self._slew_pose(dt)
        if self.mode != STAND:
            self.wanted = GaitCommand()
            if self.nav.active:
                self.nav.stop()
                lines.append(self.nav.status())
        if self.mode == STAND:
            self.since_move += dt
            if self.nav.active and self.since_line > LINK_TIMEOUT:
                # Plus rien du PC (câble, appli fermée) : le robot ne continue pas seul.
                self.nav.stop()
                self.wanted = GaitCommand()
                lines.append(self.nav.status())
            elif self.nav.active:
                # Le robot se pilote seul : pas besoin de MOVE.
                self.since_move = 0.0
                self.wanted, event = self.nav.command(self.odom, self.scanner, dt)
                if event:
                    lines.append("NAV " + event)
                    lines.append(self.nav.status())
            elif self.since_move > WATCHDOG:
                self.wanted = GaitCommand()
            r.command = self._filter(self.wanted, lines)
            if self.tilted:
                r.command = GaitCommand()  # arrêt de sécurité : trop penché
        elif self.mode == STANDING:
            self.height = min(0.0, self.height + self.sit_drop / RISE_TIME * dt)
            if self.height == 0.0:
                self.mode = STAND
        elif self.mode == SITTING:
            r.command = GaitCommand()
            if r.at_rest and r.tuck_settled:  # tous les pieds posés au neutre
                self.height = max(-self.sit_drop, self.height - self.sit_drop / RISE_TIME * dt)
                if self.height == -self.sit_drop:
                    self.mode = SIT

        elif self.mode == WAKE:
            self._wake_tick(dt)

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
            r.replace_feet = self.mode == STAND  # pas sur place seulement debout
            try:
                self.angles = r.update(dt)
            except UnreachableError:
                # On garde la dernière position atteignable.
                r.command = GaitCommand()
            if r.six is not self._walk_gait:
                lines.append(self.walk_status())  # tripode <-> vague (WAVE AUTO) : prévenir l'appli
            self._integrate(r.travel)
            self._went += math.sqrt(r.travel[0] ** 2 + r.travel[1] ** 2)
            # Pose réduite par le robot (servos en butée sinon) : prévenir l'appli.
            self._free_time = 0.0 if r.pose_scale < 0.99 else self._free_time + dt
            if r.pose_scale < 0.95 and not self.pose_limited:
                self.pose_limited = True
                lines.append("LIMIT pose 1")
            elif self.pose_limited and self._free_time > 1.5:
                self.pose_limited = False
                lines.append("LIMIT pose 0")

        # Consignes du tick envoyées aux servos, sauts adoucis (pas en marche).
        self.smooth.flush(dt, self.mode == STAND and not r.still)

        if self.imu is not None:
            self.imu.update(dt, self.robot.applied, self.body_odom(), *self._sim_feet())
            att = self.imu.attitude
            if self.mode == STAND and att.ready:
                u = self.user_pose
                self.leveler.update(dt, (att.roll, att.pitch), (u.roll, u.pitch))
                r.feel(dt, *att.rate)
                self._govern(dt, att)
                self._tilt_guard(dt, att, lines)
            else:
                self.leveler.relax(dt)
                r.time_scale = 1.0
                if self.tilted and not att.ready:
                    # Assiette inconnue (gyroscope perdu, biais remesuré) : plus d'arrêt de sécurité.
                    self.tilted = False
                    lines.append("TILT 0")
        if hasattr(self.scanner.ranger, "tick"):
            self.scanner.ranger.tick(dt)  # simulation : la personne marche
        if self.mode != OFF:  # servos relâchés : le balayage attend
            lines += self.scanner.tick(dt, self.odom, self.wanted)

        self.since_telemetry += dt
        if self.since_telemetry >= self.telemetry_period:
            lines.append(self.telemetry())
            lines.append(self.measures(self.since_telemetry))
            self.since_telemetry = 0.0
            x, y, th = self.body_odom()
            lines.append("P %d %d %d" % (round(x), round(y), round(th * 1000)))
            if self.imu is not None and self.imu.attitude.ready:
                a, lv = self.imu.attitude, self.leveler
                lines.append("I %d %d %d %d" % tuple(round(v * 1800 / math.pi) for v in (a.roll, a.pitch, lv.roll, lv.pitch)))
            # Assiette du corps par rapport aux pieds, et pente du sol (simulation) : pour la 3D.
            b = self.robot.applied
            gr, gp = getattr(self.imu, "slope", (0.0, 0.0))
            lines.append("B %d %d %d %d %d" % (tuple(round(v * 1800 / math.pi) for v in (b.roll, b.pitch, gr, gp))
                                                + (self._stance_mask(),)))
            plane = getattr(self.imu, "plane", None)
            if plane:
                lines.append("G %d %d %d %d %d" % ((round(plane[0]), round(plane[1] * 10000), round(plane[2] * 10000))
                                                    + self._ground_ref()))
            ranger = self.scanner.ranger
            if hasattr(ranger, "tick"):
                person = ranger.person
                if person:
                    lines.append("PERSON %d %d" % (round(person[0]), round(person[1])))
        self._tick_us = max(self._tick_us, ticks_diff(ticks_us(), t0))
        if getattr(self, "_prof", None) is not None:
            line = self._prof_line(dt)
            if line:
                lines.append(line)
        return "\n".join(lines) if lines else None

    def measures(self, period):
        """Ligne M : vitesse réelle et demandée (mm/s), plus petite marge d'un
        servo à sa butée (0,1°), pose et levée gardées (%), plus long tick (µs)."""
        r = self.robot
        w = self.wanted
        if self._margin_of is not self.angles:  # angles inchangés : même marge
            self._margin_of = self.angles
            self._margin = min(leg_margin(leg, self.angles[leg.name]) for leg in r.config.legs)
        margin = self._margin
        line = "M %d %d %d %d %d %d" % (
            round(self._went / period) if period else 0, round(math.sqrt(w.vx * w.vx + w.vy * w.vy)),
            round(margin * 10), round(r.pose_scale * 100), round(r.lift_scale * 100), self._tick_us)
        self._went, self._tick_us = 0.0, 0
        return line

    def telemetry(self):
        values = [self.mode, self.robot.gait.phase]
        lag = self.smooth.lag if self.smooth.busy else None
        for leg in self.robot.config.legs:
            a = self.angles[leg.name]
            for joint, v in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
                if lag is not None:
                    # Servo pas encore sur sa consigne : l'angle vraiment envoyé.
                    p = lag(joint.board, joint.channel)
                    if p is not None:
                        k = _joint(joint)
                        if k[9] != 0.0:
                            p = (p + k[10] * k[9]) / (1.0 + k[9])  # course réelle du servo (TRAVEL)
                        v = (p - k[7]) / k[8]
                values.append(round(v * 1000))
        return _T_FORMAT % tuple(values)

    def _slew_pose(self, dt):
        """La pose suivie rejoint la pose demandée, sans à-coup."""
        g, u = self.pose_goal, self.user_pose
        accel = self.smooth.accel
        if accel <= 0.0 or dt <= 0.0:
            if accel <= 0.0:
                self.user_pose = g
                self._pose_vel = [0.0] * 6
            return
        k = accel / SLEW_DEG
        vel = self._pose_vel
        vals = []
        done = True
        for i in range(6):
            x = getattr(g, POSE_AXES[i])
            e = getattr(u, POSE_AXES[i]) - x
            if e == 0.0:
                vel[i] = 0.0
            else:
                e, vel[i] = soft_step(e, vel[i], (POSE_SLEW_MM if i < 3 else POSE_SLEW_RAD) * k, dt)
                if e != 0.0:
                    done = False
            vals.append(x + e)
        self.user_pose = g if done and vel == [0.0] * 6 else BodyPose(*vals)

    # --- commandes --------------------------------------------------------

    def handle(self, line):
        """Exécute une ligne de commande et retourne la réponse."""
        parts = line.strip().split()
        if not parts:
            return None
        self.since_line = 0.0
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
                self._start_wake()
            return "OK POWER 1"
        self.mode = OFF
        self.robot.command = GaitCommand()
        self.smooth.relax()
        return "OK POWER 0"

    def cmd_stand(self):
        if self.mode in (OFF, CALIBRATION, TEST):
            self.cmd_power("1")  # se lever depuis l'arrêt alimente d'abord les servos
        if self.mode == WAKE:
            self.stand_after_wake = True  # se lèvera une fois le démarrage fini
        elif self.mode != STAND:
            self.mode = STANDING
        return "OK STAND"

    def cmd_sit(self):
        if self.mode == WAKE:
            self.stand_after_wake = False
            return "OK SIT"
        if self.mode not in (STAND, STANDING, SIT):
            return "ERR SIT : le robot n'est pas debout"
        if self.mode != SIT:
            self.mode = SITTING
        return "OK SIT"

    def cmd_move(self, vx, vy, yaw):
        if self.mode != STAND:
            return "ERR MOVE : le robot doit être debout"
        if self.nav.active:
            self.nav.stop()  # on reprend la main
            self._pending.append(self.nav.status())
        vx, vy, yaw = float(vx), float(vy), float(yaw)
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(yaw)):
            # nan ou inf : la démarche et l'odométrie ne s'en remettraient pas.
            raise ValueError("vitesse invalide")
        self.wanted = GaitCommand(vx, vy, yaw * DEG)
        self.robot.command = self._filter(self.wanted, self._pending)
        self.since_move = 0.0
        return None  # envoyé en continu : pas d'accusé de réception

    def cmd_pose(self, x, y, z, roll, pitch, yaw):
        self.pose_goal = BodyPose(
            _clamp(float(x), 20), _clamp(float(y), 20), _clamp(float(z), 15),
            _clamp(float(roll), 8) * DEG, _clamp(float(pitch), 8) * DEG,
            _clamp(float(yaw), 20) * DEG,
        )
        return None

    def cmd_gait(self, period, step_height):
        r = self.robot
        r.tripod.period = max(0.4, min(3.0, float(period)))
        # Pas plus haut que prévu par la config : le tibia toucherait la coxa.
        top = self.robot.config.step_height
        r.tripod.step_height = r.quad.step_height = max(5.0, min(top, float(step_height)))
        # À 4 pattes, chaque patte fait son pas à tour de rôle : cycle deux fois plus long.
        r.quad.period = r.wave.period = 2 * r.tripod.period
        r.wave.step_height = r.tripod.step_height
        return "OK GAIT %.2f %.0f" % (r.tripod.period, r.tripod.step_height)

    def cmd_legs(self, n):
        r = self.robot
        if int(n) != r.legs and not r.command.is_zero:
            return "ERR LEGS : arrêter le robot d'abord"
        try:
            r.set_legs(int(n))
        except ValueError as e:
            if str(e) == "4 ou 6 pattes":
                raise
            return "ERR LEGS : attendre que les pieds soient posés"
        return "OK LEGS %d" % r.legs

    def cmd_wave(self, *args):
        r = self.robot
        if args:
            what = args[0].upper()
            if what == "AUTO":
                if len(args) > 1:
                    lo, hi = WAVE_SPEED_RANGE
                    r.wave_speed = max(lo, min(hi, float(args[1])))
                r.wave_mode = WAVE_AUTO
            elif what in ("0", "1"):
                r.wave_mode = int(what)
            else:
                raise ValueError("0, 1 ou AUTO [mm/s]")
        return self.walk_status()

    def cmd_shift(self, mm):
        self.robot.wave_shift = max(0.0, min(WAVE_SHIFT_MAX, float(mm)))
        return self.walk_status()

    def cmd_sag(self, deg):
        self.robot.sag = max(0.0, min(SAG_MAX, float(deg)))
        return self.walk_status()

    def cmd_slew(self, accel):
        self.smooth.accel = max(0.0, min(SLEW_MAX, float(accel)))
        return self.walk_status()

    def cmd_walk(self):
        return self.walk_status()

    def walk_status(self):
        """Ligne WALK : réglages de la marche et démarche à 6 pattes en cours."""
        r = self.robot
        self._walk_gait = r.six
        return "WALK wave %s %.0f gait %s shift %.1f sag %.1f slew %.0f" % (
            ("0", "1", "auto")[r.wave_mode], r.wave_speed, "wave" if r.six is r.wave else "tripod", r.wave_shift,
            r.sag, self.smooth.accel)

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

    def cmd_hold(self, leg_name, joint_name, deg):
        """En pose de calibration, tient un servo à `deg` degrés de cette
        pose : pour mesurer sa course réelle au rapporteur (voir TRAVEL)."""
        joint = self._joint(leg_name, joint_name)
        if self.mode != CALIBRATION:
            return "ERR HOLD : pose de calibration d'abord (CAL)"
        name = joint_name.lower()
        base = CALIBRATION_POSE[JOINTS.index(name)]
        rad = base + _clamp(float(deg), 60) * DEG
        # Borné aux butées du servo : la réponse donne l'angle vraiment tenu.
        d = 90.0 + joint.direction * (rad * DEG_PER_RAD - joint.zero_deg) + joint.trim_deg
        d = max(joint.min_deg, min(joint.max_deg, d))
        rad = ((d - 90.0 - joint.trim_deg) / joint.direction + joint.zero_deg) / DEG_PER_RAD
        self.angles = self._calibration_angles()
        setattr(self.angles[leg_name.upper()], name, rad)
        self._send_angles()
        return "OK HOLD %s %s %.1f" % (leg_name.upper(), name, (rad - base) * DEG_PER_RAD)

    def cmd_travel(self, *args):
        """Course réelle des servos, en µs d'impulsion par degré."""
        legs = self.robot.config.legs
        if not args:
            return "TRAVEL " + json.dumps(self._travel())
        lo, hi = US_PER_DEG_RANGE
        if len(args) == 2 and args[0].upper() == "ALL":
            us = max(lo, min(hi, float(args[1])))
            for leg in legs:
                leg.coxa.us_per_deg = leg.femur.us_per_deg = leg.tibia.us_per_deg = us
            reply = "OK TRAVEL ALL %.2f" % us
        elif len(args) == 3:
            joint = self._joint(args[0], args[1])
            joint.us_per_deg = max(lo, min(hi, float(args[2])))
            reply = "OK TRAVEL %s %s %.2f" % (args[0].upper(), args[1].lower(), joint.us_per_deg)
        else:
            raise ValueError("patte articulation us, ou ALL us")
        self.robot._written = None  # tout renvoyer aux servos
        if self.mode in (CALIBRATION, TEST):
            self._send_angles()
        return reply

    def _travel(self):
        return {leg.name: [leg.coxa.us_per_deg, leg.femur.us_per_deg, leg.tibia.us_per_deg]
                for leg in self.robot.config.legs}

    def cmd_invert(self, leg_name, joint_name, on):
        joint = self._joint(leg_name, joint_name)
        joint.inverted = on == "1"
        if self.mode in (CALIBRATION, TEST):
            self._send_angles()
        return "OK INVERT %s %s %d" % (leg_name.upper(), joint_name.lower(), joint.inverted)

    def cmd_save(self):
        data = self.calibration()
        data["travel"] = self._travel()  # course réelle des servos (TRAVEL)
        imu = self._stored().get("imu")
        if imu:
            data["imu"] = imu  # réglages du gyroscope (IMU AXES, IMU ZERO) : gardés
        with open(self.calib_path, "w") as f:
            f.write(json.dumps(data))
        return "OK SAVE"

    def cmd_getcal(self):
        return "CAL " + json.dumps(self.calibration())

    def cmd_geom(self):
        return "GEOM " + json.dumps(self.robot.config.to_dict())

    def cmd_sensor(self, *args):
        sc = self.scanner
        if args and args[0].upper() == "STOP":
            lo, hi = STOP_RANGE
            sc.stop_dist = max(lo, min(hi, float(args[1])))
        elif args:
            if args[0] == "1" and not sc.present:
                return "ERR SENSOR : capteur avant absent"
            sc.enable(args[0] == "1")
        return sc.status()

    def cmd_goto(self, x, y, speed=None):
        if self.mode != STAND:
            return "ERR GOTO : le robot doit être debout"
        self._auto_sensor()
        self.nav.goto(float(x), float(y), float(speed) if speed else None)
        return self.nav.status()

    def cmd_follow(self, speed=None):
        if self.mode != STAND:
            return "ERR FOLLOW : le robot doit être debout"
        if not self.scanner.present:
            return "ERR FOLLOW : il faut le capteur avant"
        self._auto_sensor()
        self.nav.follow(float(speed) if speed else None)
        return self.nav.status()

    def cmd_nav(self, *args):
        if args and args[0].upper() == "STOP":
            self.nav.stop()
            self.wanted = GaitCommand()
        return self.nav.status()

    def cmd_person(self, x, y):
        ranger = self.scanner.ranger
        if not hasattr(ranger, "tick"):
            return "ERR PERSON : seulement en simulation"
        ranger.walk_person_to(float(x), float(y))
        return None

    def _auto_sensor(self):
        # Se déplacer seul sans regarder devant soi : le capteur s'allume.
        if self.scanner.present and not self.scanner.enabled:
            self.scanner.enable(True)
            self._pending.append(self.scanner.status())

    def cmd_imu(self, *args):
        if args:
            if self.imu is None:
                return "ERR IMU : pas de gyroscope"
            what = args[0].upper()
            att = self.imu.attitude
            if what == "CAL":
                att.restart()  # biais du gyroscope mesuré de nouveau
            elif what == "AXES":
                self.imu.axes = parse_axes("".join(args[1:]))
                att.restart()  # le biais était mesuré avec les anciens axes
                if not self._save_imu():
                    return "ERR IMU : écriture impossible dans la mémoire du robot"
            elif what == "ZERO":
                if len(args) > 1:
                    if args[1] != "0":
                        raise ValueError("ZERO, ou ZERO 0 pour l'effacer")
                    zr = zp = 0.0
                else:
                    if not att.ready:
                        raise ValueError("gyroscope pas prêt (robot immobile 1 s)")
                    # À plat, le corps a l'assiette que lui donnent les pattes : le reste est l'écart de la carte.
                    b = self.robot.applied
                    zr, zp = att.raw_roll - b.roll, att.raw_pitch - b.pitch
                    if max(abs(zr), abs(zp)) > ZERO_MAX:
                        raise ValueError("penché de plus de 10° : poser le robot à plat (ou corriger IMU AXES)")
                att.set_zero(zr, zp)
                if not self._save_imu():
                    return "ERR IMU : écriture impossible dans la mémoire du robot"
            else:
                raise ValueError("CAL, ZERO ou AXES")
        imu = self.imu
        zr, zp = (imu.attitude.zero_roll, imu.attitude.zero_pitch) if imu is not None else (0.0, 0.0)
        return "IMU present %d level %d feel %d ready %d axes %s zero %d %d" % (
            imu is not None, self.leveler.enabled, self.robot.feel_ground,
            imu is not None and imu.attitude.ready, format_axes(imu.axes if imu is not None else IMU_AXES),
            round(zr * 1800 / math.pi), round(zp * 1800 / math.pi))

    def _save_imu(self):
        """Réglages du gyroscope gardés dans le fichier de calibration, sans toucher aux trims enregistrés."""
        data = self._stored()
        att = self.imu.attitude
        data["imu"] = {"axes": format_axes(self.imu.axes), "zero": [att.zero_roll, att.zero_pitch]}
        try:
            with open(self.calib_path, "w") as f:
                f.write(json.dumps(data))
        except OSError:
            return False
        return True

    def cmd_prof(self, on):
        """PROF 1 : temps moyen (ms) de chaque partie du cycle, une ligne PROF
        par seconde ; PROF 0 pour arrêter. Sert à trouver ce qui ralentit l'ESP32."""
        if on != "1":
            for obj, name in getattr(self, "_prof_wrapped", ()):
                try:
                    delattr(obj, name)  # retour à la méthode de la classe
                except AttributeError:
                    pass
            self._prof_wrapped = []
            self._prof = None
            return "OK PROF 0"
        if getattr(self, "_prof", None) is not None:
            return "OK PROF 1"
        self._prof = {}
        self._prof_time = 0.0
        self._prof_wrapped = []
        r = self.robot
        parts = [(self, "tick", "tick"), (r, "update", "update"), (r, "solve_limited", "ik"),
                 (r, "_move_feet", "pieds"), (r, "_refresh_home", "place"),
                 (self.driver, "set_angle", "servos"), (self.driver, "set_angles", "servos"),
                 (self, "telemetry", "telem"),
                 (self, "measures", "telem")]
        for board in getattr(self.driver, "boards", ()):
            if board:
                parts.append((board, "write_block", "i2c"))  # bus seul (compris dans servos)
        if self.imu is not None:
            parts.append((self.imu, "update", "gyro"))
        parts.append((self.scanner, "tick", "capteur"))
        for obj, name, label in parts:
            if hasattr(obj, name):
                setattr(obj, name, self._timed(getattr(obj, name), label))
                self._prof_wrapped.append((obj, name))
        return "OK PROF 1"

    def _timed(self, f, label):
        prof = self._prof

        def timed(*args):
            t = ticks_us()
            out = f(*args)
            prof[label] = prof.get(label, 0) + ticks_diff(ticks_us(), t)
            return out

        return timed

    def _prof_line(self, dt):
        """Ligne PROF (une par seconde) quand la mesure est active."""
        prof = self._prof
        self._prof_time += dt
        prof["n"] = prof.get("n", 0) + 1
        if self._prof_time < 1.0:
            return None
        n = prof.pop("n")
        line = "PROF cycles %d" % n + "".join(
            " %s %.1f" % (k, v / 1000.0 / n) for k, v in sorted(prof.items()))
        try:
            import gc

            line += " memoire_libre %d" % gc.mem_free()
        except (ImportError, AttributeError):
            pass
        prof.clear()
        self._prof_time = 0.0
        return line

    def cmd_feel(self, on):
        if on == "1" and self.imu is None:
            return "ERR FEEL : pas de gyroscope"
        self.robot.feel_ground = on == "1"
        if not self.robot.feel_ground:
            self.robot.time_scale = 1.0
        return self.cmd_imu()

    def cmd_level(self, on):
        if on == "1" and self.imu is None:
            return "ERR LEVEL : pas de gyroscope"
        self.leveler.enabled = on == "1"
        return self.cmd_imu()

    def cmd_tilt(self, roll, pitch):
        if not hasattr(self.imu, "ground_roll"):
            return "ERR TILT : seulement en simulation"
        self.imu.ground_roll = _clamp(float(roll), 15) * DEG
        self.imu.ground_pitch = _clamp(float(pitch), 15) * DEG
        return "OK TILT %.0f %.0f" % (self.imu.ground_roll / DEG, self.imu.ground_pitch / DEG)

    def cmd_terrain(self, on=None):
        if not hasattr(self.imu, "terrain"):
            return "ERR TERRAIN : seulement en simulation"
        if on is not None:
            self.imu.terrain = on == "1"
        if self.imu.terrain:
            from .terrain import describe

            self._pending.append(describe())  # l'appli dessine le même terrain
        return "TERRAIN %d" % self.imu.terrain

    def _govern(self, dt, att):
        """Corps qui bascule vite (pied qui tape, trou) : la démarche ralentit
        le temps que ça se calme, puis reprend sa vitesse."""
        r = self.robot
        if not r.feel_ground:
            r.time_scale = 1.0
            return
        fr, fp = r.feel_rate  # lissée : un tick bref (dt petit) ne fait pas de pic
        rate = math.sqrt(fr * fr + fp * fp)
        goal = max(SLOW_MIN, min(1.0, 1.0 - (rate - SLOW_RATE) / SLOW_SPAN))
        if goal < r.time_scale:
            r.time_scale = goal
        else:
            r.time_scale = min(goal, r.time_scale + SLOW_RECOVER * dt)

    def _tilt_guard(self, dt, att, lines):
        """Arrêt de sécurité : corps penché de plus de TILT_STOP, le robot
        s'arrête ; il repart quand il est revenu sous TILT_OK depuis 1 s."""
        tilt = max(abs(att.roll), abs(att.pitch))
        if tilt > TILT_STOP:
            self._upright_time = 0.0
            if not self.tilted:
                self.tilted = True
                lines.append("TILT 1")
        elif self.tilted:
            self._upright_time = self._upright_time + dt if tilt < TILT_OK else 0.0
            if self._upright_time >= 1.0:
                self.tilted = False
                lines.append("TILT 0")

    def _stance_mask(self):
        """Pieds posés : bit i = i-ème patte de la config (pour la 3D)."""
        r = self.robot
        walking = r.walking_legs()
        mask = 0
        for i, leg in enumerate(r.config.legs):
            if leg.name in walking and r.offsets[leg.name][2] < 0.5:
                mask |= 1 << i
        return mask

    def body_odom(self):
        """Position du corps sur le sol : l'odométrie suit le centre de la
        démarche ; à 4 pattes le corps s'en écarte (balancement)."""
        x, y, th = self.odom
        sx, sy = self.robot.gait.shift
        c, s = math.cos(th), math.sin(th)
        return (x + c * sx - s * sy, y + s * sx + c * sy, th)

    def _ground_ref(self):
        """Un pied posé qui touche le terrain (indice dans la config) et sa
        hauteur par rapport au pied normal, en 0,1 mm : la 3D cale le
        terrain dessus (un pied allongé n'est pas le sol)."""
        air = getattr(self.imu, "air", [])
        names = self._stance_names
        if not air or len(air) != len(names):
            return (-1, 0)
        i = min(range(len(air)), key=lambda k: air[k])
        legs = [leg.name for leg in self.robot.config.legs]
        return (legs.index(names[i]), round(self._stance_z[i] * 10))

    def _sim_feet(self):
        """Pieds posés et pieds en l'air (repère corps, z par rapport au pied
        posé normal), pour le terrain de la simulation."""
        if not getattr(self.imu, "terrain", False):
            return None, ()
        r = self.robot
        targets = r.foot_targets()
        stance, swing = [], []
        self._stance_names, self._stance_z = [], []
        for n in r.walking_legs():
            x, y, z = targets[n]
            foot = (x, y, z - r.neutral[n][2])
            if r.offsets[n][2] < 0.5:
                stance.append(foot)
                self._stance_names.append(n)
                self._stance_z.append(foot[2])
            else:
                swing.append(foot)
        return stance, swing

    def cmd_obst(self, *args):
        ranger = self.scanner.ranger
        if not hasattr(ranger, "obstacles"):
            return "ERR OBST : seulement en simulation"
        if args and args[0].upper() == "ADD":
            x, y, r = float(args[1]), float(args[2]), float(args[3])
            if len(ranger.obstacles) < 40:
                ranger.obstacles.append((x, y, max(10.0, min(300.0, r))))
        elif args and args[0].upper() == "CLEAR":
            del ranger.obstacles[:]
        elif args:
            raise ValueError("ADD x y r ou CLEAR")
        return "OBST " + " ".join("%d %d %d" % (x, y, r) for x, y, r in ranger.obstacles)

    # --- calibration ------------------------------------------------------

    def calibration(self):
        legs = self.robot.config.legs
        data = {leg.name: [leg.coxa.trim_deg, leg.femur.trim_deg, leg.tibia.trim_deg] for leg in legs}
        data["invert"] = {
            leg.name: [int(j.inverted) for j in (leg.coxa, leg.femur, leg.tibia)] for leg in legs
        }
        return data

    def _stored(self):
        """Contenu du fichier de calibration ({} s'il n'y en a pas)."""
        try:
            # stat avant open : sur MicroPython (port unix), un open() qui
            # échoue peut fermer le descripteur 0 (stdin) au passage du GC.
            os.stat(self.calib_path)
            with open(self.calib_path) as f:
                data = json.loads(f.read())
        except (OSError, ValueError):
            return {}
        return data if isinstance(data, dict) else {}

    def load_calibration(self):
        data = self._stored()
        imu = data.get("imu")
        if imu and self.imu is not None:
            # Réglages du gyroscope ; absents d'une ancienne calibration : valeurs de départ.
            try:
                axes = parse_axes(imu["axes"])
                zr, zp = float(imu["zero"][0]), float(imu["zero"][1])
            except (KeyError, IndexError, TypeError, ValueError):
                pass
            else:
                self.imu.axes = axes
                self.imu.attitude.set_zero(zr, zp)
        for leg in self.robot.config.legs:
            trims = data.get(leg.name)
            if trims and len(trims) == 3:
                leg.coxa.trim_deg, leg.femur.trim_deg, leg.tibia.trim_deg = trims
            inv = data.get("invert", {}).get(leg.name)
            if inv and len(inv) == 3:
                leg.coxa.inverted, leg.femur.inverted, leg.tibia.inverted = [bool(v) for v in inv]
            # Course réelle des servos ; absente d'une ancienne calibration : valeurs de départ.
            us = data.get("travel", {}).get(leg.name)
            if us and len(us) == 3:
                lo, hi = US_PER_DEG_RANGE
                try:
                    us = [max(lo, min(hi, float(v))) for v in us]
                except (TypeError, ValueError):
                    continue
                leg.coxa.us_per_deg, leg.femur.us_per_deg, leg.tibia.us_per_deg = us

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
        p, lv = self.user_pose, self.leveler
        limit = 12 * DEG  # pose + correction d'assiette : reste dans la course des servos
        self.robot.pose = BodyPose(p.x, p.y, self.height + p.z, _clamp(p.roll + lv.roll, limit),
                                   _clamp(p.pitch + lv.pitch, limit), p.yaw)
        self.robot.base_pose = BodyPose(0.0, 0.0, self.height)
        # Assis, le corps est posé : pas d'appoint des pattes (SAG) ; il vient en se levant.
        self.robot.sag_k = 1.0 + self.height / self.sit_drop if self.sit_drop > 0.0 else 1.0

    def _send_angles(self):
        for leg in self.robot.config.legs:
            self._send_leg(leg, self.angles[leg.name])

    def _send_leg(self, leg, a):
        from .servo import ik_to_servo_deg

        for joint, value in ((leg.coxa, a.coxa), (leg.femur, a.femur), (leg.tibia, a.tibia)):
            self.smooth.set_angle(joint.board, joint.channel, ik_to_servo_deg(value, joint))

    def _zero_angles(self):
        # Angles IK qui placent chaque servo au milieu de sa course.
        return {
            leg.name: LegAngles(leg.coxa.zero_deg * DEG, leg.femur.zero_deg * DEG, leg.tibia.zero_deg * DEG)
            for leg in self.robot.config.legs
        }

    def _start_wake(self):
        # Position de départ connue seulement si les servos tenaient déjà une
        # pose (calibration, test) ; après un relâchement, elle est inconnue.
        known = self.mode != OFF
        # Butées peut-être changées (calibration), servos relâchés : tout
        # recalculer et tout renvoyer.
        self.robot._solve_key = self.robot._written = None
        self.wake_from = {n: LegAngles(a.coxa, a.femur, a.tibia) for n, a in self.angles.items()} if known else None
        self.wake_time = 0.0
        self.stand_after_wake = False
        self.mode = WAKE
        self.height = -self.sit_drop
        self.robot.command = GaitCommand()
        self.robot.reset_gait()
        self._apply_pose()
        self.wake_to = self.robot.solve()
        self._wake_tick(0.0)

    def _wake_tick(self, dt):
        self.wake_time += dt
        legs = self.robot.config.legs
        done = True
        for i, leg in enumerate(legs):
            target = self.wake_to[leg.name]
            if self.wake_from is None:
                # Pattes activées l'une après l'autre, directement en posture assise.
                if self.wake_time + 1e-9 < i * WAKE_STAGGER:
                    done = False
                    continue
                a = target
            else:
                s = min(1.0, self.wake_time / WAKE_BLEND)
                s = s * s * (3 - 2 * s)  # départ et arrivée en douceur
                if s < 1.0:
                    done = False
                b = self.wake_from[leg.name]
                a = LegAngles(*(u + (v - u) * s for u, v in (
                    (b.coxa, target.coxa), (b.femur, target.femur), (b.tibia, target.tibia))))
            self.angles[leg.name] = a
            self._send_leg(leg, a)
        if done:
            self.mode = STANDING if self.stand_after_wake else SIT

    def _calibration_angles(self):
        return {leg.name: LegAngles(*CALIBRATION_POSE) for leg in self.robot.config.legs}

    def _filter(self, wanted, lines):
        """Consigne envoyée à la démarche : l'avance est coupée si un obstacle
        barre le chemin (tourner et reculer restent possibles)."""
        sc = self.scanner
        was = sc.blocked
        blocked = sc.check(self.odom, wanted)
        if blocked != was:
            lines.append("OBSTACLE %d" % blocked)
        if blocked:
            return GaitCommand(0.0, 0.0, wanted.yaw_rate)
        return wanted

    def _integrate(self, travel):
        dx, dy, dth = travel
        x, y, th = self.odom
        mid = th + dth / 2
        c, s = math.cos(mid), math.sin(mid)
        self.odom = (x + c * dx - s * dy, y + s * dx + c * dy, th + dth)

    def _safe_solve(self):
        try:
            self._apply_pose()
            return self.robot.solve()
        except UnreachableError:
            return None


def _clamp(v, limit):
    return max(-limit, min(limit, v))
