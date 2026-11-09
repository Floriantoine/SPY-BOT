"""Compare le cœur C++ (firmware_c/src) au Controller Python, tick par tick.

Depuis la racine du dépôt :

    python3 firmware_c/test/compare.py

Compile le banc de test (g++), joue les mêmes scénarios dans les deux
versions, avec dt fixe (0,02 s), et compare à chaque tick : les 18 angles
articulaires, les angles servo envoyés (servo du capteur compris), toutes les
lignes de texte (réponses, T, M sans la durée du tick, P, B, I, S, LIMIT,
OBSTACLE, TILT, NAV...) et les échanges sur le bus I²C du gyroscope et du
capteur avant. Code de sortie 0 si tout correspond.

Scénarios :
  - base : sans capteurs (marche dans toutes les directions, poses fortes,
    réglages de la démarche, calibration, test des servos, chien de garde...) ;
  - 4 pattes : LEGS 4 / 6, marche et virages à 4 pattes, poses, assis/debout ;
  - capteurs : faux MPU6050 et VL53L1X sur un faux bus I²C (même modèle des
    puces des deux côtés). Un petit monde simulé (ici seulement) leur donne
    leurs mesures, tick par tick, d'après l'état du robot Python : pente du
    sol, chocs, bruit, obstacles, personne à suivre, mesures pas prêtes,
    lectures en échec. Les mêmes valeurs brutes sont ensuite données au C++ ;
  - gyroscope seul, puis gyroscope en panne : appareils absents ou qui
    ne répondent pas comme prévu au démarrage ;
  - gyroscope : réglages : robot bougé pendant la mesure du biais, IMU CAL,
    gyroscope qui ne répond plus puis revient.
"""

import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(ROOT, "firmware_c", "src")
sys.path.insert(0, ROOT)

from hexapod import default_config  # noqa: E402
from hexapod import imu as imu_mod  # noqa: E402
from hexapod import sensor as sensor_mod  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
from hexapod.servo import DryRunDriver  # noqa: E402

time.sleep_ms = lambda ms: None  # MicroPython (VL53L1X.__init__) : rien à attendre en test

DT = 0.02
TOL = 1e-6

# Calibration de départ (trims et un servo inversé), chargée par les deux.
CALIB = {
    "RF": [1.5, -2.0, 0.5], "RM": [0.0, 3.0, -1.0], "RR": [-0.5, 0.0, 2.0],
    "LF": [0.0, 0.0, 0.0], "LM": [2.5, -1.5, 0.0], "LR": [0.0, 1.0, -2.5],
    "invert": {"RF": [0, 0, 0], "RM": [0, 0, 0], "RR": [0, 0, 0],
               "LF": [0, 0, 0], "LM": [0, 0, 0], "LR": [0, 0, 0]},
}
NAMES = ("RF", "RM", "RR", "LF", "LM", "LR")


def scenario_base():
    s = []

    def cmd(line):
        s.append(("C", line))

    def ticks(n):
        for _ in range(n):
            s.append(("K", DT))

    def walk(line, seconds, every=10):
        n = int(round(seconds / DT))
        for i in range(n):
            if i % every == 0:
                cmd(line)
            s.append(("K", DT))

    # Avant d'alimenter : questions de l'appli, erreurs de syntaxe.
    for line in ("GEOM", "GETCAL", "SENSOR", "IMU", "PING", "FOO 1", "ping x", "MOVE 1 2 3", "SIT", "POSE a 1 1 1 1 1",
                 "POSE 1 2", "TRIM XX coxa 1", "TRIM RF knee 1", "TRIM RF femur zz", "INVERT RF", "LEGS x",
                 "LEGS 4.0", "LEGS 3", "LEGS 6", "GOTO 1 2", "GOTO 1 2 3 4", "FOLLOW", "NAV", "NAV STOP", "FEEL 1",
                 "FEEL 0", "LEVEL 1", "LEVEL 0", "TILT 1 2", "TERRAIN", "TERRAIN 1 2", "OBST", "OBST ADD 1 2 3",
                 "PERSON 1 2", "SENSOR 1", "SENSOR 0", "SENSOR STOP 45.7", "SENSOR STOP 1e9", "SENSOR STOP x",
                 "SENSOR", "GAIT 1 x", "GAIT 1.4 1_8.9", "POWER", "CAL extra", "TEST RF", "move", "Gait 1.4 18.9",
                 "POSE inf -inf nan 1e400 -0.0 +.5", "POSE 0 0 0 0 0 0"):
        cmd(line)
    ticks(10)
    cmd("POWER 1")
    ticks(100)
    cmd("GAIT 1.4 18.9")
    cmd("STAND")
    ticks(120)
    for line in ("MOVE nan 0 0", "MOVE 0 inf 0", "MOVE 0 0 -Infinity", "MOVE x nan 0"):  # refusées
        cmd(line)
    ticks(5)
    # Marche dans toutes les directions, à plusieurs vitesses.
    for line, sec in (("MOVE 60 0 0", 3), ("MOVE 150 0 0", 2.5), ("MOVE 250 0 0", 2), ("MOVE 0 100 0", 2.5),
                      ("MOVE 0 -180 0", 2), ("MOVE 0 0 40", 2.5), ("MOVE 0 0 -90", 2), ("MOVE 100 80 20", 3),
                      ("MOVE -120 30 -30", 2.5), ("MOVE -200 -200 60", 2), ("MOVE 70 -40 -15", 2),
                      ("MOVE 0 0 0", 2)):
        walk(line, sec)
    # Réglages de la démarche en marchant.
    cmd("GAIT 0.8 15")
    walk("MOVE 120 0 10", 2.5)
    cmd("GAIT 2.5 40")
    walk("MOVE 50 50 0", 3)
    cmd("GAIT 0.1 1")
    walk("MOVE -40 0 0", 1.5)
    cmd("GAIT 1.4 18.9")
    walk("MOVE 0 0 0", 1.5)
    # Poses : petites puis fortes (butées, LIMIT), à l'arrêt et en marchant.
    for pose in ("POSE 5 0 0 0 0 0", "POSE 0 10 5 3 -2 5", "POSE 20 20 -15 8 8 20", "POSE -20 -20 15 -8 -8 -20",
                 "POSE 0 0 -15 0 8 0", "POSE 30 -30 30 30 30 30"):
        cmd(pose)
        ticks(60)
        walk("MOVE 120 40 15", 2)
        walk("MOVE 0 0 0", 1)
    cmd("POSE 0 0 0 0 0 0")
    walk("MOVE 0 0 0", 3)
    # Chien de garde : plus de MOVE pendant 2 s.
    cmd("MOVE 150 0 0")
    ticks(110)
    cmd("MOVE 0 0 30")
    ticks(60)
    cmd("NAV STOP")
    ticks(60)
    cmd("FOLLOW")
    # S'asseoir en marchant.
    walk("MOVE 100 0 0", 1)
    cmd("LEGS 4")  # en marche : refusé
    cmd("GOTO 1 2 x")
    cmd("GOTO 1 2 3 4")
    cmd("SIT")
    cmd("MOVE 100 0 0")
    ticks(200)
    cmd("SIT")
    # Test des servos, calibration.
    cmd("TEST RF femur")
    ticks(170)
    cmd("TEST lm TIBIA")
    ticks(40)
    cmd("STAND")
    ticks(30)
    cmd("TEST LR coxa")
    ticks(10)
    cmd("POWER 0")
    ticks(5)
    cmd("CAL")
    ticks(5)
    for line in ("TRIM RF femur 3.25", "TRIM LM tibia -50", "TRIM rr COXA 1.25", "INVERT RR coxa 1",
                 "INVERT LF femur 1", "INVERT LF femur 0", "GETCAL", "GEOM", "SAVE"):
        cmd(line)
        ticks(2)
    cmd("TEST RR coxa")
    ticks(50)
    cmd("INVERT RR coxa 0")
    ticks(20)
    cmd("POWER 1")  # depuis une pose connue : rejoint la posture assise en douceur
    ticks(30)
    cmd("STAND")
    ticks(150)
    walk("MOVE 90 -60 25", 3)
    cmd("TRIM RF femur -6")  # debout : sans effet immédiat sur les angles
    walk("MOVE 90 -60 25", 2)
    # Relâché en marchant, puis redémarrage (position inconnue) et debout tout de suite.
    cmd("POWER 0")
    ticks(20)
    cmd("POWER 1")
    ticks(3)
    cmd("STAND")
    ticks(3)
    cmd("SIT")
    ticks(3)
    cmd("STAND")
    ticks(250)
    walk("MOVE 200 0 45", 3)
    cmd("POSE 15 -10 -10 5 5 10")
    walk("MOVE -150 100 -20", 3)
    cmd("SIT")
    ticks(250)
    cmd("POWER 1")
    cmd("POWER 0")
    ticks(5)
    cmd("STAND")
    ticks(250)
    # Au hasard : consignes, poses et réglages mélangés.
    rnd = random.Random(1234)
    for _ in range(80):
        r = rnd.random()
        if r < 0.5:
            walk("MOVE %d %d %d" % (rnd.randint(-250, 250), rnd.randint(-250, 250), rnd.randint(-90, 90)),
                 rnd.choice((0.3, 0.6, 1.0, 1.5)))
        elif r < 0.75:
            cmd("POSE %.1f %.1f %.1f %.1f %.1f %.1f" % tuple(rnd.uniform(-25, 25) for _ in range(6)))
            ticks(rnd.randint(1, 30))
        elif r < 0.85:
            cmd("GAIT %.2f %.1f" % (rnd.uniform(0.3, 3.2), rnd.uniform(3, 25)))
            ticks(rnd.randint(1, 20))
        elif r < 0.9:
            cmd("SIT")
            ticks(rnd.randint(10, 150))
            cmd("STAND")
            ticks(rnd.randint(10, 150))
        elif r < 0.95:
            ticks(rnd.randint(20, 80))  # chien de garde
        else:
            cmd("TRIM %s %s %.2f" % (rnd.choice(NAMES), rnd.choice(("coxa", "femur", "tibia")), rnd.uniform(-8, 8)))
            ticks(2)
    cmd("GETCAL")
    cmd("SIT")
    ticks(300)
    cmd("POWER 0")
    ticks(10)
    return s


class Script:
    """Construit un scénario : commandes (texte, ou fonction de l'état du robot
    Python qui rend le texte), ticks, et changements du monde simulé."""

    def __init__(self):
        self.steps = []

    def cmd(self, line):
        self.steps.append(("C", line))

    def ticks(self, n):
        self.steps.extend([("K", DT)] * n)

    def world(self, fn):
        self.steps.append(("W", fn))

    def walk(self, line, seconds, every=10):
        for i in range(int(round(seconds / DT))):
            if i % every == 0:
                self.cmd(line)
            self.steps.append(("K", DT))


def scenario_quad():
    """Marche à 4 pattes : passages 6 <-> 4, marche et virages, poses, réglages."""
    sc = Script()
    cmd, ticks, walk = sc.cmd, sc.ticks, sc.walk
    cmd("LEGS 4")  # éteint : accepté, les pattes du milieu se replieront
    cmd("LEGS 6")
    cmd("POWER 1")
    ticks(100)
    cmd("LEGS 4")  # assis : repli pendant 1 s
    ticks(80)
    cmd("STAND")
    cmd("LEGS 6")  # se lève en dépliant
    ticks(150)
    cmd("LEGS 4")
    ticks(30)
    cmd("MOVE 80 0 0")  # pendant le repli : la marche attend
    ticks(40)
    for line, sec in (("MOVE 60 0 0", 4), ("MOVE 120 0 0", 4), ("MOVE 0 80 0", 3), ("MOVE 0 -80 0", 3),
                      ("MOVE 0 0 30", 3), ("MOVE 0 0 -45", 3), ("MOVE 80 40 15", 4), ("MOVE -60 -30 -10", 3)):
        walk(line, sec)
        cmd("LEGS 6")  # en marche : refusé
    walk("MOVE 0 0 0", 3)
    cmd("LEGS 6")
    ticks(80)
    walk("MOVE 150 0 20", 3)
    walk("MOVE 0 0 0", 2)
    cmd("LEGS 4")
    ticks(5)
    cmd("LEGS 4")
    ticks(80)
    cmd("GAIT 1.0 12")
    walk("MOVE 100 0 0", 3)
    cmd("GAIT 2.0 18")
    walk("MOVE 50 50 -20", 3)
    cmd("GAIT 1.4 18.9")
    for pose in ("POSE 10 0 -5 0 0 0", "POSE 0 10 5 5 -5 10", "POSE -15 15 -10 8 8 -15"):
        cmd(pose)
        ticks(40)
        walk("MOVE 90 20 10", 2.5)
        walk("MOVE 0 0 0", 1.5)
    cmd("POSE 0 0 0 0 0 0")
    walk("MOVE 0 0 0", 2)
    cmd("MOVE 120 0 0")
    ticks(90)  # chien de garde à 4 pattes
    cmd("SIT")
    ticks(250)
    cmd("STAND")
    ticks(150)
    walk("MOVE 70 0 25", 3)
    cmd("POWER 0")  # relâché en marchant à 4 pattes
    ticks(10)
    cmd("STAND")
    ticks(250)
    walk("MOVE 100 0 0", 2)
    rnd = random.Random(99)
    for _ in range(50):
        r = rnd.random()
        if r < 0.6:
            walk("MOVE %d %d %d" % (rnd.randint(-150, 150), rnd.randint(-150, 150), rnd.randint(-60, 60)),
                 rnd.choice((0.5, 1.0, 2.0)))
        elif r < 0.75:
            cmd("POSE %.1f %.1f %.1f %.1f %.1f %.1f" % tuple(rnd.uniform(-20, 20) for _ in range(6)))
            ticks(rnd.randint(1, 30))
        elif r < 0.85:
            cmd("GAIT %.2f %.1f" % (rnd.uniform(0.5, 3.0), rnd.uniform(5, 20)))
            ticks(rnd.randint(1, 20))
        else:
            walk("MOVE 0 0 0", 2.5)
            cmd(rnd.choice(("LEGS 6", "LEGS 4")))
            ticks(rnd.randint(10, 80))
    walk("MOVE 0 0 0", 3)
    cmd("LEGS 6")
    ticks(80)
    cmd("SIT")
    ticks(250)
    return sc.steps


def scenario_devices():
    """Gyroscope et capteur avant : LEVEL, FEEL, sécurité d'inclinaison,
    obstacles, GOTO (arrivée, détour, bloqué), FOLLOW, et tout ça à 4 pattes."""
    sc = Script()
    cmd, ticks, walk, world = sc.cmd, sc.ticks, sc.walk, sc.world

    def setw(**kw):
        def fn(w, ctrl):
            for k, v in kw.items():
                setattr(w, k, v)
        return fn

    for line in ("SENSOR", "IMU", "LEVEL 1", "FEEL 1", "IMU", "SENSOR 1", "SENSOR STOP 50", "SENSOR", "GOTO 1 2",
                 "FOLLOW", "FOLLOW 50", "NAV", "NAV STOP", "LEGS 4", "LEGS 6", "TILT 1 2", "TERRAIN", "TERRAIN 1",
                 "OBST", "OBST ADD 1 2 3", "PERSON 1 2", "SENSOR 0", "SENSOR STOP 80", "FEEL 0", "LEVEL 0",
                 "SENSOR 2", "FEEL x", "LEVEL", "IMU 1"):
        cmd(line)
    ticks(70)  # biais du gyroscope mesuré (robot immobile)
    cmd("POWER 1")
    ticks(80)
    world(setw(slope=(4 * math.pi / 180, -3 * math.pi / 180)))
    cmd("STAND")
    ticks(120)
    cmd("LEVEL 1")
    cmd("FEEL 1")
    cmd("SENSOR 1")
    walk("MOVE 0 0 0", 3)  # la correction d'assiette s'installe
    walk("MOVE 120 0 0", 3)
    # Obstacle devant : la marche s'arrête, tourner et reculer restent possibles.
    world(lambda w, c: w.obstacles.append(ahead(c, 520, 0) + (60.0,)))
    walk("MOVE 120 0 0", 4)
    walk("MOVE 120 30 0", 1)
    walk("MOVE -80 0 0", 1.5)
    walk("MOVE 0 0 35", 3)
    walk("MOVE 100 0 -20", 3)
    cmd("SENSOR STOP 200")
    world(lambda w, c: w.obstacles.append(ahead(c, 450, 60) + (80.0,)))
    walk("MOVE 120 20 0", 3)
    cmd("SENSOR 0")  # coupé alors qu'il bloquait
    walk("MOVE 100 0 0", 1)
    cmd("SENSOR 1")
    cmd("SENSOR STOP 80")
    walk("MOVE 0 0 -40", 2)
    # Chocs (pied qui tape) : sol deviné, marche ralentie.
    for spike in ((0.9, -0.4, 0.0), (-0.7, 0.6, 0.1), (0.3, 1.2, -0.2), (-1.5, -0.2, 0.0)):
        world(setw(spike=spike, spike_ticks=4))
        walk("MOVE 90 -30 10", 1.2)
    world(setw(gyro_noise=120))  # vibrations fortes
    walk("MOVE 100 0 0", 3)
    world(setw(gyro_noise=12))
    # Mesures du capteur pas prêtes, lectures en échec.
    world(setw(blind_ticks=30))
    walk("MOVE 0 0 25", 1.5)
    world(setw(imu_fail_p=0.3, rng_fail_p=0.3))
    walk("MOVE 60 0 0", 2)
    world(setw(imu_fail_p=0.01, rng_fail_p=0.01))
    # Trop penché : arrêt de sécurité, puis reprise.
    world(setw(slope=(36 * math.pi / 180, 0.0)))
    walk("MOVE 100 0 0", 2)
    world(setw(slope=(14 * math.pi / 180, 0.0)))
    walk("MOVE 100 0 0", 2)
    world(setw(slope=(2 * math.pi / 180, -34 * math.pi / 180)))
    walk("MOVE 100 0 0", 1.5)
    world(setw(slope=(0.0, 0.0)))
    walk("MOVE 100 0 0", 3)
    cmd("FEEL 0")
    cmd("LEVEL 0")
    walk("MOVE 80 0 0", 1.5)
    cmd("LEVEL 1")
    cmd("FEEL 1")
    walk("MOVE 0 0 0", 1)
    # GOTO : arrivée, sans obstacle.
    world(lambda w, c: w.obstacles.clear())
    cmd(lambda c: "GOTO %.1f %.1f" % ahead(c, 500, 250))
    ticks(1500)
    # GOTO avec un obstacle sur le chemin : détour.
    cmd(lambda c: "GOTO %.1f %.1f 80" % ahead(c, 900, 0))
    world(lambda w, c: w.obstacles.append(ahead(c, 420, 0) + (90.0,)))
    ticks(2000)
    cmd("NAV")
    # Reprise en main par MOVE pendant un GOTO.
    cmd(lambda c: "GOTO %.1f %.1f" % ahead(c, -600, 300))
    ticks(60)
    walk("MOVE 0 0 0", 0.5)
    # Encerclé : le robot tourne sur place puis abandonne (stuck).
    world(lambda w, c: w.obstacles.extend(
        ahead(c, 330 * math.cos(k * math.pi / 6), 330 * math.sin(k * math.pi / 6)) + (70.0,) for k in range(12)))
    cmd(lambda c: "GOTO %.1f %.1f 0" % ahead(c, 1500, -200))
    ticks(800)
    cmd(lambda c: "GOTO %.1f %.1f" % ahead(c, 2000, 0))
    ticks(30)
    cmd("NAV STOP")
    ticks(30)
    # Suivre une personne.
    world(lambda w, c: w.obstacles.clear())
    world(lambda w, c: setattr(w, "person", ahead(c, 650, 80)))
    world(lambda w, c: setattr(w, "person_goal", ahead(c, 1600, -300)))
    cmd("FOLLOW 90")
    ticks(700)
    world(lambda w, c: setattr(w, "person_goal", ahead(c, 300, 0)))  # elle revient vers le robot
    ticks(300)
    world(lambda w, c: setattr(w, "person", ahead(c, 2500, 2500)))  # disparue
    ticks(200)
    world(lambda w, c: setattr(w, "person", ahead(c, 700, -150)))
    world(lambda w, c: setattr(w, "person_goal", ahead(c, 900, 600)))
    ticks(400)
    cmd("SIT")  # la navigation s'arrête en s'asseyant
    ticks(200)
    cmd("FOLLOW")
    cmd("GOTO 0 0")
    cmd("STAND")
    ticks(150)
    # À 4 pattes, avec le gyroscope et le capteur.
    walk("MOVE 0 0 0", 2)
    cmd("LEGS 4")
    ticks(80)
    world(setw(slope=(3 * math.pi / 180, 5 * math.pi / 180)))
    walk("MOVE 80 0 0", 4)
    walk("MOVE 40 0 25", 3)
    for spike in ((0.8, 0.3, 0.0), (-0.6, -0.9, 0.0)):
        world(setw(spike=spike, spike_ticks=5))
        walk("MOVE 70 20 0", 1.5)
    world(lambda w, c: w.obstacles.append(ahead(c, 450, 0) + (70.0,)))
    walk("MOVE 90 0 0", 3)
    walk("MOVE 0 0 -30", 2)
    world(lambda w, c: w.obstacles.clear())
    cmd(lambda c: "GOTO %.1f %.1f" % ahead(c, 300, -200))
    ticks(1200)
    world(setw(slope=(0.0, -35 * math.pi / 180)))
    walk("MOVE 60 0 0", 2)
    world(setw(slope=(0.0, 0.0)))
    walk("MOVE 0 0 0", 3)
    cmd("LEGS 6")
    ticks(80)
    # Au hasard : marche, capteur, réglages.
    rnd = random.Random(4321)
    for _ in range(40):
        r = rnd.random()
        if r < 0.5:
            walk("MOVE %d %d %d" % (rnd.randint(-150, 200), rnd.randint(-150, 150), rnd.randint(-60, 60)),
                 rnd.choice((0.5, 1.0, 2.0)))
        elif r < 0.6:
            world(lambda w, c, k=rnd.uniform(300, 700), sd=rnd.uniform(-200, 200):
                  w.obstacles.append(ahead(c, k, sd) + (60.0,)))
        elif r < 0.65:
            world(lambda w, c: w.obstacles.clear())
        elif r < 0.75:
            cmd(rnd.choice(("SENSOR 0", "SENSOR 1", "LEVEL 0", "LEVEL 1", "FEEL 0", "FEEL 1",
                            "SENSOR STOP %d" % rnd.randint(20, 320))))
            ticks(rnd.randint(1, 10))
        elif r < 0.85:
            world(setw(spike=(rnd.uniform(-1.5, 1.5), rnd.uniform(-1.5, 1.5), 0.0), spike_ticks=rnd.randint(1, 6)))
            ticks(rnd.randint(1, 20))
        else:
            cmd("POSE %.1f %.1f %.1f %.1f %.1f %.1f" % tuple(rnd.uniform(-15, 15) for _ in range(6)))
            ticks(rnd.randint(1, 20))
    cmd("SIT")
    ticks(250)
    cmd("POWER 0")
    ticks(20)
    cmd("IMU")
    cmd("SENSOR")
    return sc.steps


def scenario_imu_only():
    """Gyroscope seul (capteur avant qui n'est pas un VL53L1X : ignoré)."""
    sc = Script()
    cmd, ticks, walk, world = sc.cmd, sc.ticks, sc.walk, sc.world
    for line in ("SENSOR", "SENSOR 1", "IMU", "LEVEL 1", "FEEL 1"):
        cmd(line)
    ticks(60)
    cmd("STAND")
    ticks(300)
    cmd("FOLLOW")
    cmd(lambda c: "GOTO %.1f %.1f" % ahead(c, 400, 0))  # sans capteur : va droit au but
    ticks(800)
    world(lambda w, c: setattr(w, "slope", (0.0, 36 * math.pi / 180)))
    walk("MOVE 100 0 0", 2)
    world(lambda w, c: setattr(w, "slope", (0.0, 0.0)))
    walk("MOVE 100 0 0", 2)
    cmd("SIT")
    ticks(200)
    return sc.steps


def scenario_imu_broken():
    """Gyroscope qui refuse sa configuration : absent pour le robot."""
    sc = Script()
    for line in ("IMU", "LEVEL 1", "FEEL 1", "SENSOR", "LEVEL 0", "IMU CAL", "IMU FOO"):
        sc.cmd(line)
    sc.cmd("STAND")
    sc.ticks(250)
    sc.walk("MOVE 100 0 0", 1)
    return sc.steps


def scenario_imu_setup():
    """Réglages et pannes du gyroscope : robot bougé pendant la mesure du
    biais, IMU CAL, lectures en échec puis retour."""
    sc = Script()
    cmd, ticks, walk, world = sc.cmd, sc.ticks, sc.walk, sc.world

    def setw(**kw):
        def fn(w, ctrl):
            for k, v in kw.items():
                setattr(w, k, v)
        return fn

    # On bouge le robot pendant la première seconde : la mesure du biais recommence.
    ticks(30)
    world(setw(spike=(0.4, -0.2, 0.1), spike_ticks=4))
    ticks(40)
    cmd("IMU")  # pas encore prêt
    ticks(40)
    cmd("IMU")
    for line in ("IMU CAL", "IMU", "IMU cal extra", "IMU FOO", "IMU 1"):
        cmd(line)
    ticks(20)
    world(setw(spike=(0.0, 0.06, 0.0), spike_ticks=1))  # juste au-dessus du seuil
    ticks(60)
    cmd("IMU")
    cmd("STAND")
    ticks(300)
    world(setw(slope=(5 * math.pi / 180, 2 * math.pi / 180)))
    cmd("LEVEL 1")
    walk("MOVE 60 0 0", 2)
    cmd("IMU CAL")  # debout, en marche : recommence tant que ça bouge
    walk("MOVE 60 0 0", 2)
    walk("MOVE 0 0 0", 3)
    cmd("IMU")
    # Le gyroscope ne répond plus (fil débranché) : assiette inconnue, la
    # correction s'en va doucement ; puis il revient.
    world(setw(imu_fail_p=0.6))  # lectures ratées, rarement 5 de suite
    walk("MOVE 60 0 0", 2)
    world(setw(imu_fail_p=1.0))
    walk("MOVE 60 0 0", 1.5)
    cmd("IMU")
    world(setw(imu_fail_p=0.01))
    walk("MOVE 60 0 0", 2)
    cmd("IMU")
    # Perdu pendant un arrêt de sécurité : l'arrêt est levé (TILT 0).
    cmd("LEVEL 0")
    world(setw(slope=(0.0, 30 * math.pi / 180)))
    walk("MOVE 60 0 0", 2)
    world(setw(imu_fail_p=1.0))
    walk("MOVE 60 0 0", 1)
    world(setw(imu_fail_p=0.0, slope=(0.0, 0.0)))
    walk("MOVE 0 0 0", 2)
    cmd("IMU CAL")  # perdu pendant la mesure du biais : elle continue au retour
    ticks(10)
    world(setw(imu_fail_p=1.0))
    ticks(20)
    world(setw(imu_fail_p=0.0))
    ticks(70)
    cmd("IMU")
    return sc.steps


SCENARIOS = (
    ("base", scenario_base, {}),
    ("4 pattes", scenario_quad, {}),
    ("capteurs", scenario_devices, {"imu": "ok", "rng": "ok"}),
    ("gyroscope seul", scenario_imu_only, {"imu": "ok", "rng": "badid"}),
    ("gyroscope en panne", scenario_imu_broken, {"imu": "badinit"}),
    ("gyroscope : réglages", scenario_imu_setup, {"imu": "ok", "rng": "ok"}),
)


# =============================================================================
# Faux bus I²C : même modèle des puces que FakeBus dans host_main.cpp
# =============================================================================

IMU_ADDR = imu_mod.ADDRESS
RNG_ADDR = sensor_mod.VL53L1X.ADDRESS


class FakeBus:
    """machine.I2C de MicroPython, avec un faux MPU6050 et un faux VL53L1X.
    Chaque échange est noté (ligne X), dans le format du banc C++."""

    def __init__(self, imu="absent", rng="absent"):
        self.imu_on, self.imu_badinit, self.imu_fail = imu != "absent", imu == "badinit", False
        self.imu_raw = [0] * 7
        self.rng_on, self.rng_badid, self.rng_fail = rng != "absent", rng == "badid", False
        self.rng_ready, self.rng_status, self.rng_dist = False, 0, 0
        self.boot_polls, self.wake_polls, self.pointer = 2, 2, 0
        self.log = []

    def _add(self, head, data=b"", ok=True):
        self.log.append(head + "".join("%02x" % b for b in data) + ("" if ok else "!"))

    def _fails(self, addr):
        if addr == IMU_ADDR:
            return not self.imu_on or self.imu_fail
        if addr == RNG_ADDR:
            return not self.rng_on or self.rng_fail
        return True

    def scan(self):
        self._add("scan")
        return [a for a, on in ((IMU_ADDR, self.imu_on), (RNG_ADDR, self.rng_on)) if on]

    def writeto(self, addr, buf):
        ok = not self._fails(addr)
        self._add("w%02x:" % addr, buf, ok)
        if not ok:
            raise OSError(19)
        if addr == RNG_ADDR and len(buf) >= 2:
            reg = (buf[0] << 8) | buf[1]
            if len(buf) == 2:
                self.pointer = reg
            elif reg == 0x0086 and buf[2] == 0x01:
                self.rng_ready = False

    def readfrom(self, addr, n):
        ok = not self._fails(addr)
        self._add("r%02x:%d" % (addr, n), b"", ok)
        if not ok:
            raise OSError(19)
        d = bytearray(n)
        if addr != RNG_ADDR:
            return bytes(d)
        p = self.pointer
        if p == 0x010F:
            d[0] = 0xEE if self.rng_badid else 0xEA
            if n > 1:
                d[1] = 0xCC
        elif p == 0x00E5:
            if self.boot_polls > 0:
                self.boot_polls -= 1
            else:
                d[0] = 1
        elif p == 0x0030:
            d[0] = 0x01
        elif p == 0x0031:
            if not self.rng_ready and self.wake_polls > 0:
                self.wake_polls -= 1
                if self.wake_polls == 0:
                    self.rng_ready = True
            d[0] = 1 if self.rng_ready else 0
        elif p == 0x0089:
            d[0] = self.rng_status & 0xFF
        elif p == 0x0096:
            d[0] = (self.rng_dist >> 8) & 0xFF
            if n > 1:
                d[1] = self.rng_dist & 0xFF
        return bytes(d)

    def writeto_mem(self, addr, reg, buf):
        ok = not self._fails(addr) and not (addr == IMU_ADDR and self.imu_badinit and reg == 0x1B)
        self._add("W%02x:%02x:" % (addr, reg), buf, ok)
        if not ok:
            raise OSError(19)

    def readfrom_mem(self, addr, reg, n):
        ok = not self._fails(addr)
        self._add("R%02x:%02x:%d" % (addr, reg, n), b"", ok)
        if not ok:
            raise OSError(19)
        d = bytearray(n)
        if addr == IMU_ADDR and reg == 0x3B:
            for i in range(7):
                if 2 * i + 1 < n:
                    v = self.imu_raw[i] & 0xFFFF
                    d[2 * i], d[2 * i + 1] = v >> 8, v & 0xFF
        return bytes(d)

    def flush(self):
        line = "X" + "".join(" " + t for t in self.log)
        self.log = []
        return line


# =============================================================================
# Petit monde simulé (Python seulement) : donne aux fausses puces leurs mesures
# =============================================================================

def _int16(v):
    return max(-32768, min(32767, int(round(v))))


class World:
    def __init__(self, seed=7):
        self.rnd = random.Random(seed)
        self.obstacles = []  # (x, y, rayon) sur le sol, repère de l'odométrie
        self.person = None  # (x, y)
        self.person_goal = None
        self.person_speed = 70.0
        self.slope = (0.0, 0.0)  # pente du sol (rad)
        self.spike = (0.0, 0.0, 0.0)  # choc : vitesses en plus (rad/s, repère corps)
        self.spike_ticks = 0
        self.gyro_bias = (35, -22, 12)  # LSB, mesuré par le robot au démarrage
        self.gyro_noise = 12
        self.accel_noise = 40
        self.ready_p = 0.85
        self.blind_ticks = 0  # capteur avant : pas de mesure prête
        self.imu_fail_p = 0.01
        self.rng_fail_p = 0.01
        self.range_noise = 3
        self._prev = None

    def devices(self, ctrl, bus, dt):
        """Valeurs brutes des puces pour le prochain tick, d'après l'état du robot Python."""
        out = []
        rnd = self.rnd
        if bus.imu_on:
            b = ctrl.robot.applied
            roll, pitch, yaw = self.slope[0] + b.roll, self.slope[1] + b.pitch, ctrl.odom[2]
            prev = self._prev or (roll, pitch, yaw)
            gyro = imu_mod.euler_rates_to_body(roll, pitch, (roll - prev[0]) / dt, (pitch - prev[1]) / dt,
                                               (yaw - prev[2]) / dt)
            self._prev = (roll, pitch, yaw)
            if self.spike_ticks > 0:
                self.spike_ticks -= 1
                gyro = tuple(g + s for g, s in zip(gyro, self.spike))
            acc = imu_mod.gravity_in_body(roll, pitch)
            assert imu_mod.IMU_AXES == ((0, 1), (1, 1), (2, 1))  # carte = corps
            raw = [_int16(a * imu_mod.MPU6050.ACCEL_SCALE + rnd.randint(-self.accel_noise, self.accel_noise))
                   for a in acc]
            raw.append(rnd.randint(-2000, 2000))  # température
            raw += [_int16(g * imu_mod.MPU6050.GYRO_SCALE + bias + rnd.randint(-self.gyro_noise, self.gyro_noise))
                    for g, bias in zip(gyro, self.gyro_bias)]
            fail = int(rnd.random() < self.imu_fail_p)
            bus.imu_raw, bus.imu_fail = raw, bool(fail)
            out.append("imu %s %d" % (" ".join(str(v) for v in raw), fail))
        if bus.rng_on:
            sc = ctrl.scanner
            x, y, th = ctrl.odom
            heading = th + sc.angle * sensor_mod.DEG
            cx = x + math.cos(th) * sensor_mod.PAN_AXIS_X + math.cos(heading) * sensor_mod.CHIP_OFFSET
            cy = y + math.sin(th) * sensor_mod.PAN_AXIS_X + math.sin(heading) * sensor_mod.CHIP_OFFSET
            sim = sensor_mod.SimRanger(self.obstacles)
            if self.person:
                sim.person = self.person
            d = sim.read(cx, cy, heading)
            if d > sensor_mod.MAX_RANGE:
                status, dist = 2, 8190
            else:
                status, dist = 9, max(0, min(65535, int(round(d)) + rnd.randint(-self.range_noise, self.range_noise)))
            if self.blind_ticks > 0:
                self.blind_ticks -= 1
                ready = 0
            else:
                ready = int(rnd.random() < self.ready_p)
            fail = int(rnd.random() < self.rng_fail_p)
            bus.rng_ready, bus.rng_status, bus.rng_dist, bus.rng_fail = bool(ready), status, dist, bool(fail)
            out.append("rng %d %d %d %d" % (ready, status, dist, fail))
        return out

    def tick(self, dt):
        if self.person is None or self.person_goal is None:
            return
        (x, y), (gx, gy) = self.person, self.person_goal
        dx, dy = gx - x, gy - y
        d = math.sqrt(dx * dx + dy * dy)
        step = self.person_speed * dt
        self.person = (gx, gy) if d <= step else (x + dx / d * step, y + dy / d * step)


def ahead(ctrl, dist, side=0.0):
    """Point du sol à `dist` mm devant le robot (et `side` mm à sa gauche)."""
    x, y, th = ctrl.odom
    return (x + math.cos(th) * dist - math.sin(th) * side, y + math.sin(th) * dist + math.cos(th) * side)


def run_python(steps, calib, devices):
    """Joue le scénario en Python. Retourne ses lignes et les étapes à donner
    au C++ (commandes calculées, valeurs des fausses puces avant chaque tick)."""
    path = os.path.join(tempfile.mkdtemp(), "calib.json")
    with open(path, "w") as f:
        f.write(json.dumps(calib))
    driver = DryRunDriver()
    out, cpp = [], []
    world = World()
    bus = FakeBus(**devices)
    with_devices = bool(devices)
    ranger = imu = None
    if with_devices:  # comme main.py : capteur avant, puis gyroscope
        ranger = sensor_mod.VL53L1X.probe(bus)
        imu = imu_mod.MPU6050.probe(bus)
    ctrl = Controller(default_config(), driver, path, ranger, imu)
    if with_devices:
        out.append(bus.flush())
        out.append("H " + ctrl.scanner.status())
        out.append("H " + ctrl.cmd_imu())
    for kind, arg in steps:
        if kind == "W":
            arg(world, ctrl)
        elif kind == "C":
            line = arg(ctrl) if callable(arg) else arg
            cpp.append(("C", line))
            try:
                r = ctrl.handle(line)
            except Exception as e:  # noqa: BLE001  (comme main.py)
                r = "ERR interne : %r" % (e,)
            out.append("R " + r if r is not None else "R-")
        else:
            if with_devices:
                for d in world.devices(ctrl, bus, arg):
                    cpp.append(("D", d))
            cpp.append(("K", arg))
            try:
                lines = ctrl.tick(arg)
            except Exception as e:  # noqa: BLE001
                lines = "ERR interne : %r" % (e,)
            world.tick(arg)
            if lines:
                out.extend("L " + l for l in lines.split("\n"))
            a = ctrl.angles
            out.append("A " + " ".join(repr(v) for leg in ctrl.robot.config.legs
                                       for v in (a[leg.name].coxa, a[leg.name].femur, a[leg.name].tibia)))
            out.append("S" + "".join(" %d:%d=%r" % (b, c, d) for (b, c), d in sorted(driver.angles.items())))
            if with_devices:
                out.append(bus.flush())
    return out, cpp


def build(exe):
    files = [os.path.join(HERE, "host_main.cpp")] + [os.path.join(SRC, f) for f in
                                                      ("controller.cpp", "robot.cpp", "pyfmt.cpp", "imu.cpp", "sensor.cpp", "nav.cpp")]
    subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                           "-o", exe] + files)


def run_cpp(exe, steps, calib, devices):
    d = tempfile.mkdtemp()
    path = os.path.join(d, "calib.txt")
    with open(path, "w") as f:
        f.write(" ".join(repr(float(v)) for n in NAMES for v in calib[n]) + "\n")
        f.write(" ".join(str(int(v)) for n in NAMES for v in calib["invert"][n]) + "\n")
    script = "".join("K %r\n" % arg if kind == "K" else "%s %s\n" % (kind, arg) for kind, arg in steps)
    args = [exe, path]
    for name, mode in sorted(devices.items()):
        args += ["--" + name, mode]
    res = subprocess.run(args, input=script.encode(), stdout=subprocess.PIPE, check=True)
    return res.stdout.decode("utf-8").split("\n")[:-1]


def numbers(line):
    return [float(v) for v in line.split()[1:]]


def servo_map(line):
    out = {}
    for item in line.split()[1:]:
        k, v = item.split("=")
        out[k] = float(v)
    return out


def compare(py, cpp):
    worst = {"A": 0.0, "S": 0.0}
    errors = []
    tick = 0
    i = j = 0
    while i < len(py) or j < len(cpp):
        a = py[i] if i < len(py) else "<fin>"
        b = cpp[j] if j < len(cpp) else "<fin>"
        i += 1
        j += 1
        kind = a.split(" ", 1)[0]
        if kind == "A":
            tick += 1
            va, vb = numbers(a), numbers(b) if b.startswith("A ") else []
            if len(va) != len(vb):
                errors.append((tick, a, b))
                continue
            d = max(abs(x - y) for x, y in zip(va, vb))
            worst["A"] = max(worst["A"], d)
            if d > TOL:
                errors.append((tick, "angles : écart %.3g" % d, ""))
        elif kind == "S":
            ma, mb = servo_map(a), servo_map(b) if b.startswith("S") else {}
            if ma.keys() != mb.keys():
                errors.append((tick, a, b))
                continue
            d = max([abs(ma[k] - mb[k]) for k in ma] or [0.0])
            worst["S"] = max(worst["S"], d)
            if d > TOL:
                errors.append((tick, "servos : écart %.3g" % d, ""))
        else:
            if a.startswith("L M ") and b.startswith("L M "):
                a, b = a.rsplit(" ", 1)[0], b.rsplit(" ", 1)[0]  # sans la durée du tick
            if a != b:
                errors.append((tick, a, b))
        if len(errors) > 20:
            break
    return tick, worst, errors


def main():
    exe = os.path.join(tempfile.mkdtemp(), "host_test")
    build(exe)
    failed = 0
    total = {"A": 0.0, "S": 0.0}
    only = sys.argv[1:]
    for name, make, devices in SCENARIOS:
        if only and name not in only:
            continue
        py, steps = run_python(make(), CALIB, devices)
        cpp = run_cpp(exe, steps, CALIB, devices)
        ticks, worst, errors = compare(py, cpp)
        lines = sum(1 for l in py if l[:1] in "LRH")
        counts = {}
        for l in py:
            if l.startswith("L "):
                k = l[2:].split(" ", 1)[0]
                counts[k] = counts.get(k, 0) + 1
        print("== %s : %d commandes, %d ticks, %d lignes de texte, %d lignes I2C comparées" % (
            name, sum(1 for k, _ in steps if k == "C"), ticks, lines, sum(1 for l in py if l.startswith("X"))))
        print("   lignes : " + " ".join("%s=%d" % kv for kv in sorted(counts.items())))
        print("   écart max : angles %.3g rad, servos %.3g°" % (worst["A"], worst["S"]))
        for k in total:
            total[k] = max(total[k], worst[k])
        for tick, a, b in errors[:20]:
            print("tick %d :\n  python : %s\n  c++    : %s" % (tick, a[:300], b[:300]))
        print("   OK : identiques" if not errors else "   ÉCHEC : %d différences" % len(errors))
        failed += bool(errors)
    print("écart max (tous) : angles %.3g rad, servos %.3g°" % (total["A"], total["S"]))
    print("OK : tout est identique" if not failed else "ÉCHEC : %d scénario(s)" % failed)
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
