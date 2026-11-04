"""Vérifie que le code du robot tourne sous MicroPython (comme sur l'ESP32).

    micropython tests/micropython_smoke.py
"""

import sys

sys.path.insert(0, ".")

from hexapod import default_config  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
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
assert ctrl.handle("SAVE") == "OK SAVE"
assert ctrl.handle("GETCAL").startswith("CAL ")
assert ctrl.handle("GEOM").startswith("GEOM ")
assert len(driver.angles) == 18

# Capteur avant simulé : un obstacle droit devant arrête l'avance.
ctrl = Controller(default_config(), DryRunDriver(), "/tmp/spy-bot-smoke-calib.json", SimRanger([(400, 0, 40)]))
assert ctrl.handle("SENSOR 1") == "SENSOR present 1 on 1 stop 200"
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
assert ctrl.odom[0] < 400 - 40 - 150, ctrl.odom
print("MicroPython OK")
