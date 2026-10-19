"""Programme de l'ESP32 (MicroPython), lancé automatiquement au démarrage.

Il lit les commandes du PC sur le port série USB (une par ligne, voir
hexapod/controller.py) et renvoie la télémétrie. Au démarrage, les servos
restent relâchés jusqu'à la commande POWER 1.

Pour reprendre la main (REPL, mise à jour des fichiers) : Ctrl-C, ou
simplement `mpremote`, qui interrompt le programme tout seul.
"""

import select
import sys
import time

from hexapod import default_config
from hexapod.controller import Controller
from hexapod.servo import PCA9685Driver

LOOP_MS = 20  # 50 Hz


def main():
    driver = PCA9685Driver()
    ctrl = Controller(default_config(), driver, "calib.json")
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ""
    print("HELLO SPY-BOT")
    for address in driver.missing:
        print("ERR carte PCA9685 0x%02x absente : ses servos sont ignores" % address)
    last = time.ticks_ms()
    try:
        while True:
            start = time.ticks_ms()
            while poll.poll(0):
                ch = sys.stdin.read(1)
                if not ch:  # fin de flux (seulement en test sur PC)
                    poll.unregister(sys.stdin)
                    break
                if ch in "\r\n":
                    if buf:
                        reply = ctrl.handle(buf)
                        if reply:
                            print(reply)
                    buf = ""
                elif len(buf) < 120:
                    buf += ch
            dt = time.ticks_diff(start, last) / 1000.0
            last = start
            line = ctrl.tick(dt)
            if line:
                print(line)
            time.sleep_ms(max(0, LOOP_MS - time.ticks_diff(time.ticks_ms(), start)))
    finally:
        # Quoi qu'il arrive (Ctrl-C, erreur), on relâche les servos.
        driver.relax()


main()
