# SPY-BOT V2 : programme de l'ESP32 en C++

Cette version remplace `main.py` (MicroPython) sur l'ESP32 par un programme
C++ (Arduino, compilé avec PlatformIO). Elle parle **exactement le même
protocole série** (115200 bauds) : l'appli PC (`pc/server.py` et la page web)
marche sans aucun changement.

Le programme Python (`main.py`, `hexapod/`) ne change pas : il sert toujours
à la simulation sur PC et à la démo web.

## Ce que fait la V2

- Calibration : `CAL`, `TEST`, `TRIM`, `INVERT`, `SAVE`, `GETCAL` (gardée dans
  la mémoire NVS de l'ESP32 au lieu de `calib.json`).
- Pilotage : `POWER`, démarrage en douceur (pattes une à une), `STAND`,
  `SIT`, `MOVE` (marche tripode à 6 pattes, démarrages et arrêts doux,
  longueur de pas selon la place de chaque patte), `POSE` (réduite
  automatiquement près des butées, ligne `LIMIT pose`), `GAIT`, chien de
  garde de 1 s, `PING`, `GEOM`, `PROF`.
- Télémétrie `T`, `M`, `P`, `B` 20 fois par seconde (10 en MicroPython).
- Mêmes calculs que le Python, opération par opération : sur PC, sur un
  long scénario (marche dans toutes les directions, poses fortes,
  calibration…), les angles des servos sont identiques au bit près
  (`test/compare.py`).

## Ce qu'elle ne fait pas encore

- Gyroscope (MPU6050) : `IMU` répond `IMU present 0 level 0 feel 0`,
  `LEVEL 1` / `FEEL 1` sont refusés.
- Capteur avant (VL53L1X) : `SENSOR` répond `SENSOR present 0 on 0 stop 80`.
- Navigation (`GOTO`, `FOLLOW`) et marche à 4 pattes (`LEGS 4`) : refusées
  avec un message `ERR`.
- Commandes de la simulation (`TILT`, `TERRAIN`, `OBST`, `PERSON`) : `ERR`
  comme sur le vrai robot en MicroPython.

Différence voulue : au démarrage, les servos sont relâchés (bit « full off »
des PCA9685) jusqu'à `POWER 1`, même si l'ESP32 a seulement redémarré.

## Avant de flasher : sauver la calibration

Le flash efface MicroPython **et** `calib.json`. Récupère-le d'abord
(robot branché en USB, sur COM6 par exemple) :

```
python -m pip install mpremote pyserial
mpremote connect COM6 cp :calib.json .
```

## Compiler et flasher (Windows)

```
python -m pip install platformio
cd firmware_c
pio run
pio run -t erase  --upload-port COM6
pio run -t upload --upload-port COM6
```

`pio run` télécharge la première fois la chaîne de compilation ESP32
(quelques minutes). `erase` est nécessaire une fois, pour effacer
MicroPython. Pour voir ce que dit le robot : `pio device monitor --port COM6`
(il doit afficher `HELLO SPY-BOT`, puis `OK BOOT ...`).

## Remettre la calibration

```
python tools\restore_calib.py COM6 ..\calib.json
```

Le script envoie les `TRIM` et `INVERT` de chaque servo, puis `SAVE`, et
affiche la calibration relue (`GETCAL`). `--dry-run` montre seulement les
commandes. Ferme l'appli PC avant (un seul programme à la fois sur le port).

## Revenir à la version MicroPython

La version MicroPython est sur la branche `v1-micropython`. Effacer la
flash, réinstaller le firmware MicroPython « ESP32 / Generic »
(<https://micropython.org/download/ESP32_GENERIC/>), puis copier les
fichiers (voir aussi le README principal) :

```
git checkout v1-micropython
python -m esptool --chip esp32 --port COM6 erase_flash
python -m esptool --chip esp32 --port COM6 --baud 460800 write_flash -z 0x1000 ESP32_GENERIC-xxxx.bin
mpremote connect COM6 fs cp -r hexapod : + fs cp main.py : + fs cp calib.json : + reset
```

## Pour les développeurs

- `src/robot_config.h` est **généré** depuis le Python (géométrie, câblage,
  butées, constantes, réponse `GEOM`) : après une modification de
  `hexapod/*.py`, relancer depuis la racine du dépôt
  `python3 firmware_c/tools/gen_config.py`.
- `src/robot.*` (IK, démarche, robot), `src/controller.*` (protocole) et
  `src/pyfmt.*` (nombres écrits comme Python) se compilent aussi sur PC ;
  `src/pca9685.*` et `src/main.cpp` sont propres à l'ESP32.
- Test de comparaison avec le Python (il faut g++) :
  `python3 firmware_c/test/compare.py`.
