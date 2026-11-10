"""Remet la calibration de la version MicroPython (calib.json) dans le robot
flashé avec la version C++ (V2).

1. AVANT de flasher la V2, récupérer le fichier de la carte :
       mpremote connect COM6 cp :calib.json .
2. Flasher la V2 (voir README.md), puis :
       python restore_calib.py COM6 calib.json

Envoie une ligne TRIM et une ligne INVERT par servo (et une ligne TRAVEL si
le fichier contient la course réelle des servos), puis SAVE (la V2 garde
la calibration dans sa mémoire NVS) et affiche la réponse GETCAL.
`--dry-run` affiche seulement les commandes, sans port série.
Il faut pyserial : python -m pip install pyserial
"""

import argparse
import json
import sys
import time

LEGS = ("RF", "RM", "RR", "LF", "LM", "LR")
JOINTS = ("coxa", "femur", "tibia")
TELEMETRY = ("T ", "M ", "P ", "B ")


def commands(data):
    """Lignes TRIM / INVERT pour un calib.json (format de Controller.save)."""
    out = []
    for leg in LEGS:
        trims = data.get(leg)
        if trims and len(trims) == 3:
            for joint, value in zip(JOINTS, trims):
                # repr : la valeur exacte (un entier reste un entier, comme en Python).
                out.append("TRIM %s %s %r" % (leg, joint, value))
        inv = data.get("invert", {}).get(leg)
        if inv and len(inv) == 3:
            for joint, value in zip(JOINTS, inv):
                out.append("INVERT %s %s %d" % (leg, joint, 1 if value else 0))
        us = data.get("travel", {}).get(leg)
        if us and len(us) == 3:
            for joint, value in zip(JOINTS, us):
                out.append("TRAVEL %s %s %r" % (leg, joint, value))
    return out


def read_reply(port, timeout=2.0):
    """Prochaine ligne qui n'est pas de la télémétrie."""
    end = time.time() + timeout
    while time.time() < end:
        line = port.readline().decode("utf-8", "replace").strip()
        if line and not line.startswith(TELEMETRY):
            return line
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", help="port série du robot (ex. COM6, /dev/ttyUSB0)")
    ap.add_argument("calib", help="calib.json récupéré de la version MicroPython")
    ap.add_argument("--dry-run", action="store_true", help="afficher les commandes sans les envoyer")
    args = ap.parse_args()
    with open(args.calib) as f:
        data = json.load(f)
    lines = commands(data)
    if args.dry_run or not args.port:
        print("\n".join(lines + ["SAVE"]))
        return 0
    import serial  # pyserial

    with serial.Serial(args.port, 115200, timeout=0.2) as port:
        # L'ouverture du port redémarre l'ESP32 : on attend son HELLO.
        end = time.time() + 5.0
        while time.time() < end:
            if port.readline().decode("utf-8", "replace").startswith("HELLO"):
                break
        time.sleep(0.5)
        port.reset_input_buffer()
        ok = True
        for line in lines + ["SAVE"]:
            port.write((line + "\n").encode())
            reply = read_reply(port)
            print("%-26s -> %s" % (line, reply))
            if not reply or not reply.startswith("OK"):
                ok = False
        port.write(b"GETCAL\n")
        print(read_reply(port))
    print("Calibration restaurée." if ok else "ATTENTION : certaines commandes ont échoué.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
