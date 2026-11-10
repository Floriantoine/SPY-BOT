# SPY-BOT V2 : programme de l'ESP32 en C++

Cette version remplace `main.py` (MicroPython) sur l'ESP32 par un programme
C++ (Arduino, compilé avec PlatformIO). Elle parle **exactement le même
protocole série** (115200 bauds) : l'appli PC (`pc/server.py` et la page web)
marche sans aucun changement.

Le programme Python (`main.py`, `hexapod/`) ne change pas : il sert toujours
à la simulation sur PC et à la démo web.

## Ce que fait la V2

Tout ce que fait `main.py` sur le vrai robot :

- Calibration : `CAL`, `TEST`, `TRIM`, `INVERT`, `SAVE`, `GETCAL` (gardée dans
  la mémoire NVS de l'ESP32 au lieu de `calib.json`).
- Pilotage : `POWER`, démarrage en douceur (pattes une à une), `STAND`,
  `SIT`, `MOVE` (marche tripode à 6 pattes, démarrages et arrêts doux,
  longueur de pas selon la place de chaque patte, pied posé sans vitesse
  verticale), `POSE` (réduite
  automatiquement près des butées, ligne `LIMIT pose`), `GAIT`, chien de
  garde de 1 s, `PING`, `GEOM`, `PROF`.
- Marche à 4 pattes : `LEGS 4` / `LEGS 6` (à l'arrêt, pieds posés), pattes
  du milieu repliées et dépliées en 1 s, une patte en l'air à la fois, corps
  qui se balance dans le triangle des 3 pieds au sol.
- Marche « vague » à 6 pattes : `WAVE 0|1|AUTO [mm/s]` (une patte en l'air à
  la fois, 5 pieds au sol ; automatique : vague à petite vitesse, tripode
  au-dessus du seuil ; changement en marche à l'instant où les 6 pieds sont
  posés), `SHIFT mm` (en vague, le corps se décale à l'opposé de la patte en
  l'air ; 6 mm au départ, 0 = aucun), `SAG deg` (les pattes qui portent
  poussent un peu plus : fémur baissé de 1,5° au départ, 0 = rien), `SLEW a`
  (sauts de consigne des servos adoucis, a °/s² au plus, 1500 au départ,
  0 = aucun ; les consignes de la marche passent telles quelles), `WALK` (réglages de la marche, ligne `WALK ...` envoyée aussi quand
  la démarche change).
- Gyroscope MPU6050 (0x68, même bus I²C que les PCA9685), cherché au
  démarrage : biais mesuré pendant la première seconde (robot immobile ; la
  mesure recommence tant qu'on le bouge, `IMU CAL` la refait), filtre
  d'assiette, `IMU`, `IMU AXES +y-x+z` (orientation de la carte) et
  `IMU ZERO` (zéro à plat), gardés dans la mémoire NVS à part des trims,
  lectures perdues signalées (`ERR I2C gyroscope : n lectures perdues`) et
  assiette ignorée après 5 lectures ratées de suite, jusqu'à ce qu'il
  réponde ; `LEVEL 1|0` (corps gardé horizontal sur une
  pente), `FEEL 1|0` (sol deviné : un pied qui tape s'arrête, un pied
  au-dessus d'un trou s'allonge, la marche ralentit si le corps bascule
  vite), arrêt de sécurité au-delà de 20° (lignes `TILT 1` / `TILT 0`),
  télémétrie `I`.
- Capteur avant VL53L1X (0x29, même bus), cherché au démarrage, sur le servo
  de balayage (carte 0x41, voie 11) : `SENSOR`, `SENSOR 1|0`, `SENSOR STOP
  mm`, mesures `S`, arrêt devant un obstacle (`OBSTACLE 1|0` : avancer est
  coupé, tourner et reculer restent possibles).
- Navigation : `GOTO x y [vitesse]` (en contournant les obstacles vus),
  `FOLLOW [vitesse]`, `NAV [STOP]`, lignes `NAV arrived|stuck|off|...`,
  position `P` (odométrie). Arrêt (`NAV off`) si plus aucune ligne n'arrive
  du PC pendant 5 s.
- Au démarrage, mêmes lignes que `main.py` : `HELLO SPY-BOT`, `OK BOOT ...`,
  `SENSOR present ...`, `IMU present ...`, cartes PCA9685 absentes.
- Télémétrie `T`, `M`, `P`, `B` (et `I`) 20 fois par seconde (10 en MicroPython).
- Mêmes calculs que le Python, opération par opération : sur PC, sur de
  longs scénarios, les angles des servos sont identiques au bit près
  (`test/compare.py`, voir plus bas).

## Ce qu'elle ne fait pas (comme MicroPython sur le vrai robot)

- Commandes de la simulation (`TILT`, `TERRAIN`, `OBST`, `PERSON`) : `ERR
  ... seulement en simulation`, comme `main.py` sur le robot. Pas de lignes
  `G` ni `PERSON`, pente du sol à 0 dans `B`.

Pas encore essayé sur le robot : les pilotes du MPU6050 et du VL53L1X (ceux
du Python non plus). Ils envoient exactement la même suite de registres que
les pilotes Python (vérifié par `test/compare.py` sur un faux bus I²C), mais
le vrai comportement des puces (temps de démarrage, mesures invalides,
orientation de la carte du gyroscope) reste à vérifier sur le robot.

Premier essai du gyroscope, dans l'ordre (appli, onglet **Corps**, ou lignes
`I roulis tangage ...` en 0,1°) :

1. Robot posé à plat, immobile 2 s après le démarrage : `IMU` doit répondre
   `IMU present 1 ... ready 1 axes +x+y+z zero 0 0` et les lignes `I` arrivent.
   Pas de ligne `I`, ou `ERR I2C gyroscope` : câblage.
2. Servos relâchés, pencher le robot à la main : côté **gauche levé** =
   roulis **positif**, **nez baissé** = tangage **positif**. Sinon :
   `IMU AXES` (boutons **Carte : ¼ de tour** et **Carte : retournée**), sans
   reflasher. Roulis près de ±180° : carte à l'envers.
3. Pencher vite de 10° et tenir : la valeur doit y aller tout droit et y
   rester. Si elle part d'abord à l'envers, recommencer l'étape 2.
4. À plat : `IMU ZERO` (**Mettre à zéro**) si les angles ne sont pas à 0.
5. Seulement alors `STAND` puis `LEVEL 1`. Le corps qui penche tout seul
   jusqu'à 10° : un axe est encore faux, `LEVEL 0`.
6. `FEEL 1` en dernier. `SENSOR 1` : environ 10 lignes `S` par seconde.

Différences voulues :

- Au démarrage, les servos sont relâchés (bit « full off » des PCA9685)
  jusqu'à `POWER 1`, même si l'ESP32 a seulement redémarré.
- Le capteur avant garde au plus 56 mesures (il n'en a jamais plus de 41 :
  une toutes les 0,1 s pendant 4 s).

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
  butées, constantes de la marche, du gyroscope, du capteur et de la
  navigation, réponse `GEOM`) : après une modification de `hexapod/*.py`,
  relancer depuis la racine du dépôt `python3 firmware_c/tools/gen_config.py`.
- Le cœur se compile aussi sur PC, sans matériel : `src/robot.*` (IK,
  démarches à 6 et 4 pattes, sol deviné), `src/imu.*` (assiette,
  auto-équilibrage, pilote MPU6050), `src/sensor.*` (balayage, obstacles,
  pilote VL53L1X), `src/nav.*` (GOTO, FOLLOW), `src/controller.*`
  (protocole), `src/pyfmt.*` (nombres écrits comme Python). Le matériel est
  derrière de petites interfaces : `driver.h` (servos), `i2c_bus.h` (bus du
  gyroscope et du capteur). `src/pca9685.*`, `src/wire_bus.*` et
  `src/main.cpp` sont propres à l'ESP32.
- Test de comparaison avec le Python (il faut g++) :
  `python3 firmware_c/test/compare.py` (ou seulement certains scénarios :
  `python3 firmware_c/test/compare.py capteurs "4 pattes"`). Scénarios :
  sans capteurs ; marche à 4 pattes ; marche en vague (changements de
  démarche en marchant, avec `LEGS 4`, des poses fortes, puis avec `LEVEL`,
  `FEEL` et le capteur) ; gyroscope et capteur avant (faux
  MPU6050 et VL53L1X sur un faux bus I²C, nourris par un petit monde simulé :
  pente, chocs, bruit, obstacles, personne à suivre, mesures pas prêtes,
  lectures en échec) ; gyroscope seul ; gyroscope qui ne répond pas ;
  réglages et pannes du gyroscope (robot bougé pendant la mesure du biais,
  `IMU CAL`, `IMU AXES`, `IMU ZERO`, lectures perdues puis retrouvées,
  liaison avec le PC perdue pendant `FOLLOW` et `GOTO`) ; réglages du
  gyroscope relus au démarrage. À
  chaque tick : angles, servos (celui du capteur compris), toutes les lignes
  de texte et tous les échanges I²C doivent être identiques.
- `sh firmware_c/test/check_arduino.sh` compile tout `src/` avec les macros
  d'Arduino-ESP32 (noms comme `HALF_PI`, `sq`, `B1`, `LINE_MAX`...).
