# SPY-BOT

Hexapode 6 pattes piloté par cinématique inverse, sur ESP32, avec une interface PC en 3D.

- **Robot** : un ESP32 sous MicroPython et 2 cartes PCA9685 (18 servos). Il calcule lui-même l'IK et la démarche.
- **PC** : une page web locale qui affiche le robot en 3D et permet de l'alimenter, le lever, le piloter (vitesse, pas de côté, rotation sur place), bouger le corps et calibrer les servos.

```
 navigateur  ⇄  pc/server.py  ⇄  USB série  ⇄  ESP32 (main.py)  ⇄  I²C  ⇄  2 × PCA9685  →  18 servos
```

L'IK tourne sur l'ESP32 et non sur le PC : la marche reste fluide même si la liaison a des à-coups, et le robot s'arrête tout seul (pieds au sol) s'il ne reçoit plus d'ordre pendant 1 s.

## Matériel et câblage

Géométrie reprise de [SmallpTsai/hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697), la carte LinkIt 7697 étant remplacée par un ESP32.

| ESP32 | PCA9685 (les deux cartes) |
| --- | --- |
| GPIO21 | SDA |
| GPIO22 | SCL |
| 3V3 | VCC |
| GND | GND (commun avec l'alimentation des servos) |

1ʳᵉ carte en `0x40` (pattes avant et milieu gauche), 2ᵉ carte en `0x41` (pont A0 soudé : milieu droite et pattes arrière). Les servos sont alimentés séparément (5-6 V, plusieurs ampères), jamais par l'ESP32.

| Patte | Carte | Canaux coxa / fémur / tibia |
| --- | --- | --- |
| Avant droite (RF) | 0x40 | 5 / 6 / 7 |
| Avant gauche (LF) | 0x40 | 2 / 3 / 4 |
| Milieu gauche (LM) | 0x40 | 8 / 9 / 10 |
| Milieu droite (RM) | 0x41 | 2 / 3 / 4 |
| Arrière gauche (LR) | 0x41 | 5 / 6 / 7 |
| Arrière droite (RR) | 0x41 | 8 / 9 / 10 |

Autre câblage ou autres broches : `WIRING` dans `hexapod/config.py`, `sda`/`scl` dans `PCA9685Driver` (`hexapod/servo.py`).

### Dimensions

Toutes les longueurs sont dans `DIMENSIONS`, en haut de `hexapod/config.py` (en mm ; actuellement l'araignée V1.5 dessinée dans Fusion 360). Si tes pièces changent, change uniquement ces valeurs : la posture debout, la longueur de pas et la vue 3D s'adaptent toutes seules. Mesure d'axe à axe :

- `root_to_coxa` : bord du châssis → axe du servo de coxa (affichage seulement)
- `coxa` : axe coxa → axe fémur
- `femur` : axe fémur → axe tibia
- `tibia` : axe tibia → contact du pied au sol
- `mount_*`, `corner_angle` : position et orientation des axes de coxa par rapport au centre du châssis
- `stand_height` : hauteur des axes de fémur au-dessus du sol, debout

La vue 3D affiche les vraies pièces de la V1.5 (`pc/web/models/*.stl`, exportées de Fusion en position montée : le corps dans le repère robot, puis coxa, fémur et tibia d'une patte chacun dans le repère de son articulation, servos au milieu). Sans ces fichiers, elle dessine un robot générique à la bonne taille.

Les replis du tibia qui le feraient toucher la coxa, la flasque ou la batterie sont dans `TIBIA_FOLD_LIMITS` ; les tests vérifient que la marche, la position assise et les pattes repliées restent en dessous.

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

Pour piloter depuis un téléphone sur le même Wi-Fi que le PC, lance `python -m pc.server --lan` et ouvre l'adresse affichée (`http://192.168.x.x:8000`) sur le téléphone. Attention : toute personne sur ce Wi-Fi peut alors piloter le robot. Le téléphone tenu en paysage passe en mode pilote (3D et joysticks en plein écran), et **Ajouter à l'écran d'accueil** l'installe comme une application.

Si le câble USB saute, le serveur rouvre le port tout seul dès que l'ESP32 revient ; le temps de réponse du robot s'affiche à côté du bouton **Connecter**.

## Montage et premier essai

Le guide pas à pas est dans [docs/MONTAGE.md](docs/MONTAGE.md) : assemblage, câblage, quel servo sur quel canal, vérification servo par servo, centrage des palonniers et calibration. L'onglet **Montage** de l'interface montre le câblage sur le modèle 3D.

## Pilotage

![Interface](docs/img/pilotage.png)

- **Joystick gauche** : avancer, reculer et se déplacer de côté, avec une vitesse proportionnelle à l'inclinaison.
- **Joystick droit** : tourner sur place.
- Clavier : Z/Q/S/D pour se déplacer, A/E pour tourner, Maj pour aller à pleine vitesse, Espace pour s'arrêter, L pour se lever ou s'asseoir, Échap pour relâcher les servos.
- Manette (Xbox, PlayStation...) : stick gauche pour se déplacer, stick droit pour tourner, A pour se lever, B pour s'asseoir.

Les onglets règlent la vitesse maximale (avec les préréglages **Lent**, **Normal** et **Rapide**), la démarche, la position et l'inclinaison du corps, le câblage et la calibration. Le journal montre les échanges avec le robot et permet de taper une commande à la main.

- **Démarrage en douceur** : à la mise sous tension, les pattes s'activent l'une après l'autre (moins d'appel de courant), puis le robot se lève.
- **Servos en butée** : un servo qui arrive au bout de sa course passe en rouge dans la 3D, avec son étiquette carte · canal.
- **Longueur de pas selon la place de chaque patte** : au démarrage, le robot calcule jusqu'où chaque pied peut aller dans chaque direction (servos à 3° de leurs butées, pattes voisines sans se toucher). La consigne n'est réduite que pour la patte qui a le moins de place : 100 mm/s en ligne droite (67 avant), 46°/s en tournant sur place (24 avant). Avec une pose du corps forte, il revient au pas prudent de la config.
- **Pieds replacés selon la pose** : quand le corps est baissé, penché ou décalé (Corps, ou l'auto-équilibrage), chaque pied se pose plus loin ou plus près du corps, là où ses servos gardent autant de marge qu'au neutre. Il y va à son prochain pas ; à l'arrêt, le robot fait un pas sur place si un pied est à plus de 8 mm de sa place (pas en se levant ni en s'asseyant).
- **Pose trop forte pour marcher** : si la pose du corps (Corps, ou l'auto-équilibrage) ajoutée au pas ferait arriver un servo en butée ou toucher deux pièces, le robot fait d'abord des pas moins hauts (jusqu'à la moitié), puis réduit la pose juste ce qu'il faut ; elle revient doucement dès que c'est possible. L'appli le signale (« Pose du corps réduite »). Le rouge ne devrait donc plus apparaître en marchant.
- **Traces des pieds** (bouton en haut à droite de la vue) : le sol défile sous le robot et chaque pied y laisse sa trace, verte au sol, bleue en l'air. Un pied qui glisse laisse un trait au lieu d'un point. Le défilement est calculé d'après les pieds posés (il suit exactement la démarche), sans capteur.
- **Animations** : révérence, danse, pompes... et un éditeur pour créer les tiennes à partir des poses de l'onglet **Corps**. Elles sont enregistrées dans le navigateur.
- **Marche à 4 pattes** (onglet **Pilotage**, **Démarche**) : les pattes du milieu se replient et les 4 pattes des coins avancent une à une, le corps se décalant avant chaque pas pour rester en équilibre. Plus lent qu'à 6 pattes ; à changer robot arrêté.
- **Servo à l'envers** : si une patte réelle part dans le sens opposé à la 3D pendant **Tester ce servo**, **Inverser le sens** corrige ce servo (enregistré avec les trims).
- **Calibration** : **Exporter** télécharge les trims dans un fichier, **Importer** les recharge (par exemple après avoir reflashé l'ESP32), puis **Enregistrer** les garde dans le robot.

## Capteur avant (VL53L1X)

Capteur de distance VL53L1X sur un servo de balayage (MG92B, carte 0x41,
canal 11), branché sur le même bus I²C que les PCA9685 (adresse 0x29, VIN sur
le 3,3 V de l'ESP32). Au démarrage, le robot dit s'il l'a trouvé ; dans
l'onglet **Pilotage**, la carte « Capteur avant » l'active ou le coupe, règle
la distance d'arrêt et montre une vue radar. Actif, il balaye ±60° (resserré
autour de la direction suivie en marche) et coupe l'avance si un obstacle est
dans le couloir des pattes à moins de la distance d'arrêt : tourner et reculer
restent possibles. En simulation, le capteur est simulé et des obstacles
virtuels (cylindres) se placent d'un clic sur le sol. Le pilote du vrai
capteur (`hexapod/sensor.py`) n'a pas encore été essayé sur le robot.

### Se déplacer seul

Carte « Seul » de l'onglet Pilotage :
- **Aller à un point** puis clic sur le sol : le robot y va en contournant ce
  que voit le capteur (il se tourne toujours vers où il va, pour le voir). Si
  le point est inatteignable, il s'arrête au bout de 10 s.
- Toucher les joysticks ou **Stop** reprend la main.
- Le suivi (commande `FOLLOW`) existe côté robot mais n'est pas encore dans
  l'interface.

## Gyroscope (MPU6050)

MPU6050 vissé sur plaque_bas, à l'arrière droite (x −38,5 à −17,5, y −30 à
−14), sur le même bus I²C (AD0 à la masse : 0x68), alimenté en 3,3 V par
l'ESP32. Le robot doit rester immobile 1 s au démarrage (mesure du biais du
gyroscope). Onglet **Corps**, carte « Gyroscope » : roulis et tangage mesurés
(horizon artificiel) et **Garder le corps horizontal**, qui incline le corps
(jusqu'à 10°) pour compenser une pente. En simulation, deux curseurs penchent
le sol virtuel, et **Terrain accidenté** pose le robot sur un sol d'ondulations,
de cailloux (jusqu'à 3 cm) et de trous (`hexapod/terrain.py`). Sans capteur
sous les pieds, le robot pose ses pieds sur le plan qu'il croit être le sol :
il repose en vrai sur ses pieds posés, penche selon le terrain sous eux, et
un pied au-dessus d'un trou reste en l'air. La 3D dessine le terrain. Premier essai sur le robot : sur sol plat, une pose de roulis
+5° doit se lire +5° ; sinon, corriger `IMU_AXES` dans `hexapod/imu.py`
(orientation de la carte). Pilote pas encore essayé sur le robot.

## Organisation du code

| Fichier | Rôle |
| --- | --- |
| `hexapod/leg_ik.py` | IK et FK d'une patte 3 DDL (solution « genou en haut »). |
| `hexapod/body_ik.py` | IK du corps : translation et roulis/tangage/lacet, pieds fixes au sol. |
| `hexapod/gait.py` | Démarches tripode (6 pattes) et pas à pas (4 pattes) pilotées par `vx`, `vy`, `yaw_rate`, arrêt toujours pieds posés. |
| `hexapod/servo.py` | Angle IK → angle servo (milieu, sens, trim, butées) et pilote PCA9685 pour MicroPython. |
| `hexapod/controller.py` | États du robot (relâché, assis, debout, calibration) et protocole texte avec le PC. |
| `hexapod/config.py` | Dimensions, posture debout, câblage. |
| `main.py` | Boucle de l'ESP32 à 50 Hz. |
| `pc/server.py` | Serveur local de l'interface PC, entre le navigateur et le port série. |
| `pc/web/` | Page web : vue 3D (three.js), joysticks, montage, calibration, animations (`anim.js`). |
| `docs/MONTAGE.md` | Guide de montage et de câblage. |
| `sim/simulate.py` | Ancienne simulation matplotlib (optionnelle). |

Le code de `hexapod/` n'utilise que ce qui existe aussi dans MicroPython : il tourne pareil sur l'ESP32 et sur le PC.

## Tests

```bash
python -m unittest -v                     # PC
micropython tests/micropython_smoke.py    # même code sous MicroPython
```

## Protocole série

Une commande par ligne à 115200 bauds, documentée en tête de `hexapod/controller.py` : `POWER 1|0`, `STAND`, `SIT`, `MOVE vx vy yaw`, `POSE x y z roll pitch yaw`, `GAIT période hauteur`, `LEGS 6|4`, `CAL`, `TEST patte articulation`, `TRIM patte articulation degrés`, `INVERT patte articulation 1|0`, `SAVE`, `GETCAL`, `GEOM`. Le robot renvoie `OK …`, `ERR …` et une ligne de télémétrie `T …` 20 fois par seconde.
