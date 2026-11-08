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
from hexapod.imu import MPU6050
from hexapod.sensor import VL53L1X
from hexapod.servo import PCA9685Driver

LOOP_MS = 20  # 50 Hz


def reset_cause():
    """Pourquoi l'ESP32 a (re)démarré : aide à repérer les chutes de tension."""
    try:
        import machine

        cause = machine.reset_cause()
    except (ImportError, AttributeError):
        return "?"
    for name in ("PWRON_RESET", "HARD_RESET", "WDT_RESET", "DEEPSLEEP_RESET", "SOFT_RESET", "BROWNOUT_RESET"):
        if getattr(machine, name, None) == cause:
            return name
    return str(cause)


def read_commands(ctrl, poll, buf):
    """Lit les caractères reçus du PC et exécute chaque ligne complète."""
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
    return buf


def full_speed():
    """ESP32 à 240 MHz (MicroPython démarre souvent à 160) : le calcul de
    l'IK de 18 servos tient alors plus facilement dans les 20 ms d'un cycle."""
    try:
        import machine

        machine.freq(240000000)
    except (ImportError, AttributeError, ValueError):
        pass


def main():
    full_speed()
    driver = PCA9685Driver()
    # Capteur avant (VL53L1X) sur le même bus I²C, s'il est branché.
    ranger = VL53L1X.probe(driver.i2c)
    # Gyroscope MPU6050 (0x68), même bus. Robot immobile au démarrage : biais mesuré pendant 1 s.
    imu = MPU6050.probe(driver.i2c)
    ctrl = Controller(default_config(), driver, "calib.json", ranger, imu)
    ctrl.telemetry_period = 0.1  # 10 par seconde : laisse le temps de calcul aux servos
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ""
    print("HELLO SPY-BOT")
    print("OK BOOT " + reset_cause())
    print(ctrl.scanner.status())
    print(ctrl.cmd_imu())
    for address in driver.missing:
        print("ERR carte PCA9685 0x%02x absente : ses servos sont ignores" % address)
    last = time.ticks_ms()
    reported_errors = 0
    reported_at = last
    try:
        while True:
            start = time.ticks_ms()
            try:
                buf = read_commands(ctrl, poll, buf)
                dt = time.ticks_diff(start, last) / 1000.0
                last = start
                line = ctrl.tick(dt)
                if line:
                    print(line)
            except Exception as e:  # noqa: BLE001
                # Une erreur ne doit jamais arrêter le robot en silence :
                # on la signale au PC et la boucle continue.
                print("ERR interne : %r" % (e,))
                last = start
            if driver.errors != reported_errors and time.ticks_diff(start, reported_at) > 1000:
                reported_errors = driver.errors
                reported_at = start
                print("ERR I2C : %d écritures perdues (câbles SDA/SCL, parasites ?)" % reported_errors)
            time.sleep_ms(max(0, LOOP_MS - time.ticks_diff(time.ticks_ms(), start)))
    finally:
        # Quoi qu'il arrive (Ctrl-C, erreur), on relâche les servos.
        driver.relax()


main()
