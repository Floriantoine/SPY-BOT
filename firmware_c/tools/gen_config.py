"""Génère firmware_c/src/robot_config.h à partir du code Python (hexapod/).

À relancer (depuis la racine du dépôt) après toute modification de
hexapod/config.py, robot.py, gait.py ou controller.py :

    python3 firmware_c/tools/gen_config.py

Les nombres sont recopiés avec repr() (aller-retour exact) : le C++ calcule
avec exactement les mêmes constantes que le Python.
"""

import inspect
import json
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, ROOT)

from hexapod import config as cfg  # noqa: E402
from hexapod import controller as ctl  # noqa: E402
from hexapod import gait as gait_mod  # noqa: E402
from hexapod import robot as rob  # noqa: E402
from hexapod import sensor as sensor_mod  # noqa: E402
from hexapod import servo as servo_mod  # noqa: E402

OUT = os.path.join(ROOT, "firmware_c", "src", "robot_config.h")


def f(v):
    """Littéral double C++ exact."""
    v = float(v)
    if math.isinf(v) or math.isnan(v):
        raise ValueError(v)
    r = repr(v)
    if "." not in r and "e" not in r:
        r += ".0"
    return r


def c_string(s):
    out = []
    for ch in s:
        if ch in '"\\':
            out.append("\\" + ch)
        else:
            out.append(ch)
    return '"' + "".join(out) + '"'


def geom_template(c):
    """Réponse GEOM exacte de Python ; le sens de chaque servo (qui change
    avec INVERT) est remplacé par %d (patte par patte, coxa fémur tibia)."""
    d = c.to_dict()
    for i, leg in enumerate(d["legs"]):
        for j, joint in enumerate(leg["joints"]):
            joint[1] = "@DIR%d@" % (3 * i + j)
    text = "GEOM " + json.dumps(d)
    assert "%" not in text
    for k in range(3 * len(d["legs"])):
        text = text.replace('"@DIR%d@"' % k, "%d", 1)
    assert "@DIR" not in text
    return text


def main():
    c = cfg.default_config()
    legs = c.legs
    names = [leg.name for leg in legs]
    trig = rob.Hexapod(c)  # pour les positions neutres, exactement comme Python
    L = []
    w = L.append
    w("// Fichier GÉNÉRÉ par firmware_c/tools/gen_config.py : ne pas modifier à la main.")
    w("// Source : hexapod/config.py, robot.py, gait.py, controller.py, servo.py.")
    w("#pragma once")
    w("")
    w("#define NLEGS %d" % len(legs))
    w("#define NJOINTS (3 * NLEGS)")
    w("")
    w("// --- géométrie (config.py) ---")
    w("static const double CFG_COXA = %s;" % f(c.coxa_length))
    w("static const double CFG_FEMUR = %s;" % f(c.femur_length))
    w("static const double CFG_TIBIA = %s;" % f(c.tibia_length))
    w("static const double CFG_STANCE_RADIUS = %s;" % f(c.stance_radius))
    w("static const double CFG_STANCE_HEIGHT = %s;" % f(c.stance_height))
    w("static const double CFG_SIT_DROP = %s;" % f(c.sit_drop))
    w("static const double CFG_STEP_HEIGHT = %s;" % f(c.step_height))
    w("static const double CFG_MAX_STRIDE = %s;" % f(c.max_stride))
    w("static const int CFG_I2C_ADDRESSES[%d] = {%s};" % (len(cfg.I2C_ADDRESSES), ", ".join("0x%02x" % a for a in cfg.I2C_ADDRESSES)))
    w("#define CFG_NBOARDS %d" % len(cfg.I2C_ADDRESSES))
    w("")
    w("struct JointDef { int board, channel; double zero_deg; int direction; double min_deg, max_deg; };")
    w("struct LegDef {")
    w("    const char* name;")
    w("    double mount_x, mount_y, mount_angle, cos_a, sin_a;  // cos/sin calculés par Python")
    w("    double neutral_x, neutral_y, neutral_z;")
    w("    int middle;    // patte du milieu (sangle de la batterie)")
    w("    int tripod_a;  // trépied A (RF, LM, RR)")
    w("    JointDef joint[3];  // coxa, fémur, tibia")
    w("};")
    w("static const LegDef CFG_LEGS[NLEGS] = {")
    for leg in legs:
        p = trig.neutral[leg.name]
        ca, sa = math.cos(leg.mount_angle), math.sin(leg.mount_angle)
        js = ", ".join("{%d, %d, %s, %d, %s, %s}" % (j.board, j.channel, f(j.zero_deg), j.direction, f(j.min_deg), f(j.max_deg))
                       for j in (leg.coxa, leg.femur, leg.tibia))
        w('    {"%s", %s, %s, %s, %s, %s, %s, %s, %s, %d, %d, {%s}},' % (
            leg.name, f(leg.mount_x), f(leg.mount_y), f(leg.mount_angle), f(ca), f(sa),
            f(p[0]), f(p[1]), f(p[2]), leg.name in cfg.MIDDLE_LEGS, leg.name in gait_mod.TRIPOD_A, js))
    w("};")
    w("// Voisines dans le sens trigo (robot.AROUND), en indices de CFG_LEGS.")
    w("static const int CFG_AROUND[NLEGS] = {%s};" % ", ".join(str(names.index(n)) for n in rob.AROUND))
    w("")
    w("// --- repli du tibia (robot.py, en radians, marge comprise) ---")
    w("#define FOLD_POINTS %d" % len(rob._FOLD_RAD))
    w("static const double FOLD_RAD[FOLD_POINTS][2] = {%s};" % ", ".join("{%s, %s}" % (f(a), f(b)) for a, b in rob._FOLD_RAD))
    w("static const double BATTERY_RAD[2] = {%s, %s};" % tuple(f(v) for v in rob._BATTERY_RAD))
    w("static const double STRAP_RAD[2] = {%s, %s};" % tuple(f(v) for v in rob._STRAP_RAD))
    w("static const double RAD_45 = %s;" % f(rob._RAD_45))
    w("static const double RAD_55 = %s;" % f(rob._RAD_55))
    w("static const double HALF_PI = %s;" % f(rob._HALF_PI))
    w("static const double MINUS_HALF_PI = %s;" % f(rob._MINUS_HALF_PI))
    w("static const double DEG_PER_RAD = %s;" % f(rob.DEG_PER_RAD))
    w("static const double SERVO_DEG_PER_RAD = %s;" % f(servo_mod.DEG_PER_RAD))
    w("static const double PY_PI = %s;" % f(math.pi))
    w("")
    w("// --- robot.py ---")
    for name in ("ACCEL", "YAW_ACCEL", "DECEL", "YAW_DECEL", "STOP_TOL", "TUCK_TIME", "POSE_RECOVER",
                 "SERVO_MARGIN", "FOLD_MARGIN", "NEIGHBOUR_LIMIT", "HOME_STEP", "HOME_REPLACE", "HOME_PERIOD",
                 "HOME_POSE_MM", "REACH_MARGIN", "REACH_COXA", "REACH_MAX", "REACH_POSE", "LIFT_MIN", "LIFT_STEP"):
        w("static const double %s = %s;" % (name, f(getattr(rob, name))))
    w("static const double HOME_RANGE_LO = %s;" % f(rob.HOME_RANGE[0]))
    w("static const double HOME_RANGE_HI = %s;" % f(rob.HOME_RANGE[1]))
    w("#define REACH_DIRS %d" % rob.REACH_DIRS)
    w("#define BISECT %d" % rob.BISECT)
    w("")
    w("// --- gait.py ---")
    g = gait_mod.TripodGait()
    w("static const double GAIT_PERIOD = %s;  // TripodGait() par défaut" % f(g.period))
    w("")
    w("// --- controller.py ---")
    for name in ("RISE_TIME", "WAKE_STAGGER", "WAKE_BLEND", "WATCHDOG", "TELEMETRY_PERIOD", "TEST_DURATION",
                 "TEST_AMPLITUDE", "DEG"):
        w("static const double %s = %s;" % (name, f(getattr(ctl, name))))
    w("static const double CALIBRATION_POSE[3] = {%s, %s, %s};" % tuple(f(v) for v in ctl.CALIBRATION_POSE))
    w("")
    w("// --- sensor.py (capteur absent : seulement l'état affiché) ---")
    w("static const double STOP_DEFAULT = %s;" % f(sensor_mod.STOP_DEFAULT))
    w("static const double STOP_RANGE_LO = %s;" % f(sensor_mod.STOP_RANGE[0]))
    w("static const double STOP_RANGE_HI = %s;" % f(sensor_mod.STOP_RANGE[1]))
    w("")
    w("// --- servo.py (PCA9685Driver) ---")
    sig = inspect.signature(servo_mod.PCA9685Driver.__init__).parameters
    for name in ("sda", "scl", "center_us", "us_per_deg", "min_us", "max_us"):
        w("static const double SERVO_%s = %s;" % (name.upper(), f(sig[name].default)))
    w("static const double PCA_FREQ = %s;" % f(inspect.signature(servo_mod.PCA9685.__init__).parameters["freq"].default))
    w("")
    w("// Réponse GEOM de Python, sens des 18 servos en %d (patte, puis coxa fémur tibia).")
    w("static const char GEOM_FORMAT[] =")
    text = geom_template(c)
    for i in range(0, len(text), 100):
        w("    " + c_string(text[i:i + 100]))
    w("    ;")
    w("")
    with open(OUT, "w", encoding="utf-8") as fh:
        fh.write("\n".join(L))
    print("écrit", os.path.relpath(OUT, ROOT))


if __name__ == "__main__":
    main()
