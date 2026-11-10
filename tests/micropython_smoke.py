"""Vérifie que le code du robot tourne sous MicroPython (comme sur l'ESP32).

    micropython tests/micropython_smoke.py
"""

import sys

sys.path.insert(0, ".")

from hexapod import default_config  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
from hexapod.imu import SimIMU  # noqa: E402
from hexapod.sensor import SimRanger  # noqa: E402
from hexapod.servo import DryRunDriver  # noqa: E402

driver = DryRunDriver()
ctrl = Controller(default_config(), driver, "/tmp/spy-bot-smoke-calib.json")


def run(seconds):
    for _ in range(int(seconds / 0.02)):
        ctrl.tick(0.02)


assert ctrl.handle("POWER 1") == "OK POWER 1"
ctrl.handle("STAND")
run(4)
assert ctrl.mode == "stand", ctrl.mode
for cmd in ("MOVE 80 0 0", "MOVE 0 40 0", "MOVE 0 0 30", "MOVE 50 0 -20"):
    for _ in range(10):
        ctrl.handle(cmd)
        run(0.1)
assert ctrl.handle("MOVE 0 0 0") is None
run(2)
assert ctrl.handle("LEGS 4") == "OK LEGS 4", "LEGS 4"
run(1.5)
for cmd in ("MOVE 80 0 0", "MOVE 0 -40 0", "MOVE 0 0 30"):
    for _ in range(15):
        ctrl.handle(cmd)
        run(0.1)
ctrl.handle("MOVE 0 0 0")
run(3)
assert ctrl.handle("LEGS 6") == "OK LEGS 6", "LEGS 6"
run(1.5)
# Marche en vague, décalage du corps, retour en tripode en marchant.
assert ctrl.handle("WAVE 1") == "WALK wave 1 20 gait tripod shift 6.0 sag 1.5 slew 1500", "WAVE 1"
assert "shift 10.0" in ctrl.handle("SHIFT 10"), "SHIFT"
assert "sag 2.0" in ctrl.handle("SAG 2"), "SAG"
ctrl.handle("SAG 1.5")
for _ in range(40):
    ctrl.handle("MOVE 20 5 3")
    run(0.1)
assert ctrl.handle("WALK") == "WALK wave 1 20 gait wave shift 10.0 sag 1.5 slew 1500", ctrl.handle("WALK")
ctrl.handle("WAVE AUTO")
for _ in range(40):
    ctrl.handle("MOVE 80 0 0")
    run(0.1)
assert ctrl.handle("WALK") == "WALK wave auto 20 gait tripod shift 10.0 sag 1.5 slew 1500", ctrl.handle("WALK")
ctrl.handle("WAVE 0")
ctrl.handle("SHIFT 6")
ctrl.handle("MOVE 0 0 0")
run(3)
ctrl.handle("POSE 5 -5 10 5 -5 10")
run(1)
ctrl.handle("SIT")
run(4)
assert ctrl.mode == "sit", ctrl.mode
assert ctrl.handle("TEST LF femur") == "OK TEST LF femur"
run(4)
assert ctrl.mode == "cal", ctrl.mode
assert ctrl.handle("CAL") == "OK CAL"
assert ctrl.handle("TRIM RR coxa 2.5").startswith("OK")
assert ctrl.handle("HOLD RF femur 45") == "OK HOLD RF femur 45.0", "HOLD"
assert ctrl.handle("TRAVEL RF femur 10.5") == "OK TRAVEL RF femur 10.50", "TRAVEL"
run(0.5)
assert ctrl.handle("TRAVEL").startswith("TRAVEL {"), "TRAVEL ?"
assert ctrl.handle("SLEW 800").endswith("slew 800"), "SLEW"
assert ctrl.handle("SAVE") == "OK SAVE"
assert ctrl.handle("GETCAL").startswith("CAL ")
assert ctrl.handle("GEOM").startswith("GEOM ")
assert len(driver.angles) == 18

# Capteur avant simulé : un obstacle droit devant arrête l'avance.
ctrl = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-smoke-calib.json", SimRanger([(400, 0, 40)]))
assert ctrl.handle("SENSOR 1") == "SENSOR present 1 on 1 stop 80"
ctrl.handle("STAND")
run(4)
out = []
for _ in range(80):
    ctrl.handle("MOVE 80 0 0")
    for _ in range(5):
        line = ctrl.tick(0.02)
        if line:
            out += line.split("\n")
assert "OBSTACLE 1" in out, "obstacle"
assert ctrl.odom[0] < 400 - 40 - 156, ctrl.odom
print("MicroPython OK")

# Gyroscope simulé : auto-équilibrage sur une pente.
imu = SimIMU()
ctrl = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-smoke-calib.json", None, imu)
ctrl.handle("STAND")
run(4)
assert ctrl.handle("TILT 5 0") == "OK TILT 5 0"
assert ctrl.handle("LEVEL 1") == "IMU present 1 level 1 feel 0 ready 1 axes +x+y+z zero 0 0"
run(4)
assert abs(imu.attitude.roll) < 0.02, imu.attitude.roll
print("MicroPython IMU OK")

# Terrain accidenté simulé : le robot repose sur ses pieds posés.
assert ctrl.handle("TERRAIN 1") == "TERRAIN 1"
assert ctrl.handle("FEEL 1") == "IMU present 1 level 1 feel 1 ready 1 axes +x+y+z zero 0 0"
for _ in range(10):
    ctrl.handle("MOVE 100 0 0")
    run(0.5)
assert imu.plane is not None
print("MicroPython terrain OK")
