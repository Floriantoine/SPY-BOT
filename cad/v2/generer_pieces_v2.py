"""Génère les pièces V2 du hexapode (STL, en mm) dans cad/v2/stl/.

Pièces conçues pour s'ajouter aux pièces hexapod-v2-7697 imprimées à 150 % :
- manchon_servo : ramène le logement servo agrandi (35,7 x 19,5) à la taille
  réelle d'un MG92B (22,8 x 12), avec les trous de vis de la bride du servo.
- support_electronique : plateau ESP32 + 2 PCA9685, vissé sur les 4 trous
  prévus à l'origine dans body_top (entraxe 60 x 51 mm à 150 %).
- verrou_connecteurs : barrette qui plaque les prises des 16 servos sur une PCA9685.
- clip_cables : pince à câbles servo vissable (M3) le long des pattes.

Toutes les cotes réglables sont en tête de fichier. Dépendances :
pip install manifold3d trimesh numpy
Usage : python cad/v2/generer_pieces_v2.py
"""

import os

import numpy as np
import trimesh
from manifold3d import Manifold

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stl")
SEG = 48  # segments par cercle

# --- Servo MG92B (mesures fabricant, à vérifier au pied à coulisse) ---------
SERVO_L, SERVO_W = 22.8, 12.0  # corps du servo
SERVO_HOLE_PITCH = 27.8  # entraxe des trous de la bride
SERVO_CLEAR = 0.2  # jeu par côté autour du corps

# --- Logement servo des pièces à 150 % (mesuré dans les STL : 23,8 x 13 x 3 mm à 100 %) ---
POCKET_L, POCKET_W, PLATE_T = 23.8 * 1.5, 13.0 * 1.5, 3.0 * 1.5
FIT = 0.15  # jeu par côté entre manchon et logement
LIP_T, LIP_OVER = 1.2, 1.5  # rebord posé sur la plaque, côtés courts seulement
PILOT_D = 1.7  # avant-trou pour vis auto-taraudeuse M2 (vis de la bride)

# --- Plateau électronique ---------------------------------------------------
TRAY_X, TRAY_Y, TRAY_T = 90.0, 116.0, 2.5
TRAY_Y0 = -10.0  # décalage du plateau par rapport au centre des trous de fixation
MOUNT_X, MOUNT_Y, MOUNT_D = 60.0, 51.0, 3.3  # trous de body_top (2,7 mm : vis M3 auto-taraudeuse)
PCA_L, PCA_W = 62.2, 25.4  # carte PCA9685 (Adafruit et clones)
PCA_HOLE_X, PCA_HOLE_Y = 55.9, 19.05  # entraxe des trous de fixation de la carte
PCA_CENTERS = [(-25.0, 12.0), (25.0, 12.0)]  # 0x41 à gauche, 0x40 à droite ; grand côté selon y
STANDOFF_D, STANDOFF_H, STANDOFF_HOLE = 6.0, 6.0, 2.2  # vis M2.5 auto-taraudeuse
ESP_L, ESP_W = 55.0, 28.5  # ESP32 DevKit 38 broches (30 broches : 51,5 x 28,5)
ESP_CENTER = (0.0, -48.0)
ESP_H = 12.0  # hauteur du dessous de la carte (laisse passer les broches)
TIE_W, TIE_L = 4.0, 2.2  # fente pour collier de serrage

# --- Verrou des prises servo ------------------------------------------------
PLUG_H = 14.0  # hauteur du haut des prises servo au-dessus du circuit
LOCK_BEAM_W, LOCK_BEAM_T = 9.0, 3.0
LOCK_HOLE = 2.8  # passage vis M2.5 (vis longue jusque dans l'entretoise du plateau)

# --- Clip à câbles ----------------------------------------------------------
CLIP_LEN = 10.0
CABLE_W, CABLE_H = 8.5, 4.0  # nappe de 3 câbles servo à plat (3 x 3 fils)
CLIP_WALL, CLIP_SLIT = 1.6, 2.5
CLIP_SCREW = 3.3


def box(x, y, z, cx=0.0, cy=0.0, z0=0.0):
    return Manifold.cube([x, y, z], center=True).translate([cx, cy, z0 + z / 2])


def cyl(d, h, cx=0.0, cy=0.0, z0=0.0):
    return Manifold.cylinder(h, d / 2, d / 2, SEG).translate([cx, cy, z0])


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def manchon_servo():
    outer = box(POCKET_L - 2 * FIT, POCKET_W - 2 * FIT, PLATE_T)
    lip = box(POCKET_L - 2 * FIT + 2 * LIP_OVER, POCKET_W - 2 * FIT, LIP_T, z0=PLATE_T)
    body = outer + lip
    hole = box(SERVO_L + 2 * SERVO_CLEAR, SERVO_W + 2 * SERVO_CLEAR, 20, z0=-5)
    pilots = [cyl(PILOT_D, 20, cx=s * SERVO_HOLE_PITCH / 2, z0=-5) for s in (-1, 1)]
    return body - hole - union(pilots)


def support_electronique():
    plate = box(TRAY_X, TRAY_Y, TRAY_T, cy=TRAY_Y0)
    cuts = []
    mounts = [(sx * MOUNT_X / 2, sy * MOUNT_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
    for x, y in mounts:
        cuts.append(cyl(MOUNT_D, 10, x, y, -1))
        cuts.append(cyl(6.2, 10, x, y, TRAY_T - 1.0))  # lamage tête de vis
    adds = []
    for cx, cy in PCA_CENTERS:
        # Évidement sous la carte (allège et laisse passer les câbles).
        cuts.append(box(PCA_W - 9, PCA_L - 16, 10, cx, cy, -1))
        for sx in (-1, 1):
            for sy in (-1, 1):
                x, y = cx + sx * PCA_HOLE_Y / 2, cy + sy * PCA_HOLE_X / 2
                adds.append(cyl(STANDOFF_D, TRAY_T + STANDOFF_H, x, y))
                cuts.append(cyl(STANDOFF_HOLE, 20, x, y, 0.8))
    ex, ey = ESP_CENTER
    cuts.append(box(ESP_L - 16, ESP_W - 8, 10, ex, ey, -1))
    for s in (-1, 1):
        # Appui sous chaque bout de la carte, avec butée extérieure.
        x_in = ex + s * (ESP_L / 2 - 3.0)
        adds.append(box(6.0, ESP_W - 4, TRAY_T + ESP_H, x_in, ey))
        adds.append(box(2.0, ESP_W - 4, TRAY_T + ESP_H + 2.5, ex + s * (ESP_L / 2 + 1.2), ey))
        # Fentes de colliers pour plaquer la carte.
        for sy in (-1, 1):
            cuts.append(box(TIE_L, TIE_W, 10, ex + s * (ESP_L / 2 - 9.0), ey + sy * (ESP_W / 2 + 3.0), -1))
    # Fentes de colliers en bord de plateau pour les faisceaux.
    for x in (-TRAY_X / 2 + 4, TRAY_X / 2 - 4):
        for y in (TRAY_Y0 + TRAY_Y / 2 - 12, TRAY_Y0 - TRAY_Y / 2 + 12):
            cuts.append(box(TIE_L, TIE_W, 10, x, y, -1))
    return (plate + union(adds)) - union(cuts)


def verrou_connecteurs():
    pillars = [cyl(6.0, PLUG_H + LOCK_BEAM_T, 0, s * PCA_HOLE_X / 2) for s in (-1, 1)]
    beam = box(LOCK_BEAM_W, PCA_HOLE_X + 6.0, LOCK_BEAM_T, z0=PLUG_H)
    holes = [cyl(LOCK_HOLE, 50, 0, s * PCA_HOLE_X / 2, -5) for s in (-1, 1)]
    return (union(pillars) + beam) - union(holes)


def clip_cables():
    ow, oh = CABLE_W + 2 * CLIP_WALL, CABLE_H + 2 * CLIP_WALL
    body = box(ow, CLIP_LEN, oh, z0=0)
    tab = box(9.0, CLIP_LEN, 2.4, cx=ow / 2 + 4.0)
    channel = box(CABLE_W, CLIP_LEN + 2, CABLE_H, z0=CLIP_WALL)
    slit = box(CLIP_SLIT, CLIP_LEN + 2, CLIP_WALL + 2, cx=-ow / 4, z0=oh - CLIP_WALL - 1)
    screw = cyl(CLIP_SCREW, 10, ow / 2 + 4.5, 0, -1)
    return (body + tab) - channel - slit - screw


PIECES = {
    "manchon_servo": manchon_servo,
    "support_electronique": support_electronique,
    "verrou_connecteurs": verrou_connecteurs,
    "clip_cables": clip_cables,
}


def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, build in PIECES.items():
        tm = to_trimesh(build())
        tm.export(os.path.join(OUT_DIR, name + ".stl"))
        ext = tm.extents
        print("{:22s} {:5.1f} x {:5.1f} x {:4.1f} mm  {:5.1f} cm3  étanche={}".format(
            name, ext[0], ext[1], ext[2], tm.volume / 1000, tm.is_watertight))


if __name__ == "__main__":
    main()
