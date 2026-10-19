# SPY-BOT

Hexapode 6 pattes piloté par cinématique inverse, sur ESP32, avec une interface PC en 3D.

- **Robot** : un ESP32 sous MicroPython et 2 cartes PCA9685 (18 servos). Il calcule lui-même l'IK et la démarche.
- **PC** : une page web locale qui affiche le robot en 3D et permet de l'alimenter, le lever, le piloter (vitesse, pas de côté, rotation sur place), bouger le corps et calibrer les servos.

```
 navigateur  ⇄  pc/server.py  ⇄  USB série  ⇄  ESP32 (main.py)  ⇄  I²C  ⇄  2 × PCA9685  →  18 servos
```

L'IK tourne sur l'ESP32 et non sur le PC : la marche reste fluide même si la liaison a des à-coups, et le robot s'arrête tout seul (pieds au sol) s'il ne reçoit plus d'ordre pendant 0,5 s.

## Matériel et câblage

Géométrie et câblage repris de [SmallpTsai/hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697), la carte LinkIt 7697 étant remplacée par un ESP32.

| ESP32 | PCA9685 (les deux cartes) |
| --- | --- |
| GPIO21 | SDA |
| GPIO22 | SCL |
| 3V3 | VCC |
| GND | GND (commun avec l'alimentation des servos) |

Carte droite en `0x40`, carte gauche en `0x41` (pont A0 soudé). Les servos sont alimentés séparément (5-6 V, plusieurs ampères), jamais par l'ESP32.

| Patte | Carte | Canaux coxa / fémur / tibia |
| --- | --- | --- |
| Avant droite (RF) | 0x40 | 5 / 6 / 7 |
| Milieu droite (RM) | 0x40 | 2 / 3 / 4 |
| Arrière droite (RR) | 0x40 | 8 / 9 / 10 |
| Avant gauche (LF) | 0x41 | 5 / 6 / 7 |
| Milieu gauche (LM) | 0x41 | 2 / 3 / 4 |
| Arrière gauche (LR) | 0x41 | 8 / 9 / 10 |

Autre câblage ou autres broches : `WIRING` dans `hexapod/config.py`, `sda`/`scl` dans `PCA9685Driver` (`hexapod/servo.py`).

### Dimensions

Toutes les longueurs sont dans `DIMENSIONS`, en haut de `hexapod/config.py` (valeurs du robot d'origine, en mm). Si tes pièces sont agrandies, change uniquement ces valeurs : la posture debout, la longueur de pas et la vue 3D s'adaptent toutes seules. Mesure d'axe à axe :

- `root_to_coxa` : bord du châssis → axe du servo de coxa
- `coxa` : axe coxa → axe fémur
- `femur` : axe fémur → axe tibia
- `tibia` : axe tibia → bout du pied
- `mount_*` : position des fixations des pattes par rapport au centre du châssis

## Installation

Sur le PC (Python 3.9 ou plus récent) :

```bash
pip install -r requirements.txt
```

### 1. Installer MicroPython sur l'ESP32 (une seule fois)

Télécharge le firmware « ESP32 / Generic » sur <https://micropython.org/download/ESP32_GENERIC/> (fichier `.bin`), puis, avec le bon port (`COM3` sous Windows, `/dev/ttyUSB0` sous Linux, `/dev/cu.usbserial-…` sous macOS) :

```bash
esptool.py --chip esp32 --port COM3 erase_flash
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 ESP32_GENERIC-xxxx.bin
```

### 2. Copier le programme du robot

Depuis la racine du dépôt :

```bash
mpremote connect COM3 fs cp -r hexapod : + fs cp main.py : + reset
```

À refaire après chaque modification de `hexapod/` ou `main.py` (par exemple après avoir changé `DIMENSIONS`). Au démarrage l'ESP32 lance `main.py`, et les servos restent relâchés.

### 3. Lancer l'interface

Ferme `mpremote` (un seul programme peut ouvrir le port série), puis :

```bash
python -m pc.server
```

Ouvre <http://localhost:8000>, choisis le port de l'ESP32 et clique sur **Connecter**. L'option **Simulation** fait tourner exactement le même code sans robot, pour essayer l'interface.

## Premier essai

1. Robot posé sur une boîte, pattes dans le vide, alimentation des servos branchée.
2. **Pose de calibration** : chaque servo va au milieu de sa course. Pour chaque patte, règle les trims jusqu'à ce que le robot ait la même pose que le modèle 3D (coxa dans l'axe de la patte, fémur relevé d'environ 15°, tibia perpendiculaire au fémur), puis **Enregistrer** (les trims sont gardés dans l'ESP32). Si un servo tourne dans le mauvais sens, change son `direction` dans `config.py`.
3. **Activer les servos** puis **Se lever**, toujours sur la boîte, pour vérifier les mouvements.
4. Pose le robot au sol, **Se lever**, puis pilote :

| Touche | Action |
| --- | --- |
| Z / S (ou ↑ / ↓) | avancer / reculer |
| Q / D | pas de côté gauche / droite |
| A / E (ou ← / →) | tourner sur place à gauche / à droite |
| Espace | stop |

Les curseurs règlent la vitesse, la vitesse de rotation, la pose du corps (hauteur, roulis, tangage, lacet) et la démarche. **Relâcher** coupe tous les servos.

## Organisation du code

| Fichier | Rôle |
| --- | --- |
| `hexapod/leg_ik.py` | IK et FK d'une patte 3 DDL (solution « genou en haut »). |
| `hexapod/body_ik.py` | IK du corps : translation et roulis/tangage/lacet, pieds fixes au sol. |
| `hexapod/gait.py` | Démarche tripode pilotée par `vx`, `vy`, `yaw_rate`, arrêt toujours pieds posés. |
| `hexapod/servo.py` | Angle IK → angle servo (milieu, sens, trim, butées) et pilote PCA9685 pour MicroPython. |
| `hexapod/controller.py` | États du robot (relâché, assis, debout, calibration) et protocole texte avec le PC. |
| `hexapod/config.py` | Dimensions, posture debout, câblage. |
| `main.py` | Boucle de l'ESP32 à 50 Hz. |
| `pc/server.py`, `pc/web/` | Interface PC : serveur local, page 3D (three.js). |
| `sim/simulate.py` | Ancienne simulation matplotlib (optionnelle). |

Le code de `hexapod/` n'utilise que ce qui existe aussi dans MicroPython : il tourne pareil sur l'ESP32 et sur le PC.

## Tests

```bash
python -m unittest -v                     # PC
micropython tests/micropython_smoke.py    # même code sous MicroPython
```

## Protocole série

Une commande par ligne à 115200 bauds, documentée en tête de `hexapod/controller.py` : `POWER 1|0`, `STAND`, `SIT`, `MOVE vx vy yaw`, `POSE x y z roll pitch yaw`, `GAIT période hauteur`, `CAL`, `TRIM patte articulation degrés`, `SAVE`, `GETCAL`, `GEOM`. Le robot renvoie `OK …`, `ERR …` et une ligne de télémétrie `T …` 20 fois par seconde.
