"""Vérifie que le code du robot tourne sous MicroPython (comme sur l'ESP32).

    micropython tests/micropython_smoke.py
"""

import sys

sys.path.insert(0, ".")

from hexapod import default_config  # noqa: E402
from hexapod.controller import Controller  # noqa: E402
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
print("MicroPython OK")
