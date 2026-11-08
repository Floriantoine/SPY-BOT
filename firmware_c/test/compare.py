"""Compare le cœur C++ (firmware_c/src) au Controller Python, tick par tick.

Depuis la racine du dépôt :

    python3 firmware_c/test/compare.py

Compile le banc de test (g++), joue le même long scénario (marche dans
toutes les directions, poses fortes, réglages de la démarche, calibration,
test des servos, chien de garde...) dans les deux versions, avec dt fixe
(0,02 s), et compare à chaque tick : les 18 angles articulaires, les angles
servo envoyés et toutes les lignes de texte (réponses, T, M sans la durée du
tick, P, B, LIMIT). Code de sortie 0 si tout correspond.
"""

import json
import os
import random
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(ROOT, "firmware_c", "src")
sys.path.insert(0, ROOT)

from hexapod import default_config  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
from hexapod.servo import DryRunDriver  # noqa: E402

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


def scenario():
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
    cmd("LEGS 4")  # en marche : refusé pareil (à l'arrêt, hors du périmètre de la version C)
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


def run_python(steps, calib):
    path = os.path.join(tempfile.mkdtemp(), "calib.json")
    with open(path, "w") as f:
        f.write(json.dumps(calib))
    driver = DryRunDriver()
    ctrl = Controller(default_config(), driver, path)
    out = []
    for kind, arg in steps:
        if kind == "C":
            try:
                r = ctrl.handle(arg)
            except Exception as e:  # noqa: BLE001  (comme main.py)
                r = "ERR interne : %r" % (e,)
            out.append("R " + r if r is not None else "R-")
        else:
            lines = ctrl.tick(arg)
            if lines:
                out.extend("L " + l for l in lines.split("\n"))
            a = ctrl.angles
            out.append("A " + " ".join(repr(v) for leg in ctrl.robot.config.legs
                                       for v in (a[leg.name].coxa, a[leg.name].femur, a[leg.name].tibia)))
            out.append("S" + "".join(" %d:%d=%r" % (b, c, d) for (b, c), d in sorted(driver.angles.items())))
    return out


def build(exe):
    files = [os.path.join(HERE, "host_main.cpp")] + [os.path.join(SRC, f) for f in
                                                      ("controller.cpp", "robot.cpp", "pyfmt.cpp")]
    subprocess.check_call(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror", "-ffp-contract=off",
                           "-o", exe] + files)


def run_cpp(exe, steps, calib):
    d = tempfile.mkdtemp()
    path = os.path.join(d, "calib.txt")
    with open(path, "w") as f:
        f.write(" ".join(repr(float(v)) for n in NAMES for v in calib[n]) + "\n")
        f.write(" ".join(str(int(v)) for n in NAMES for v in calib["invert"][n]) + "\n")
    script = "".join("C %s\n" % arg if kind == "C" else "K %r\n" % arg for kind, arg in steps)
    res = subprocess.run([exe, path], input=script.encode(), stdout=subprocess.PIPE, check=True)
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
    steps = scenario()
    exe = os.path.join(tempfile.mkdtemp(), "host_test")
    build(exe)
    py = run_python(steps, CALIB)
    cpp = run_cpp(exe, steps, CALIB)
    ticks, worst, errors = compare(py, cpp)
    lines = sum(1 for l in py if l[:1] in "LR")
    print("%d commandes, %d ticks, %d lignes de texte comparées" % (sum(1 for k, _ in steps if k == "C"), ticks, lines))
    print("écart max : angles %.3g rad, servos %.3g°" % (worst["A"], worst["S"]))
    for tick, a, b in errors[:20]:
        print("tick %d :\n  python : %s\n  c++    : %s" % (tick, a[:300], b[:300]))
    print("OK : identiques" if not errors else "ÉCHEC : %d différences" % len(errors))
    return 0 if not errors else 1


if __name__ == "__main__":
    sys.exit(main())
