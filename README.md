# SPY-BOT

Hexapode 6 pattes piloté par cinématique inverse (IK).

## Matériel visé (choix par défaut)

- 6 pattes à 3 servos chacune (coxa, fémur, tibia), soit 18 servos type MG90S/MG996R.
- Raspberry Pi + 2 cartes PCA9685 (adresses I²C `0x40` et `0x41`) : pattes droites sur la carte 0, gauches sur la carte 1.
- Dimensions par défaut : coxa 50 mm, fémur 80 mm, tibia 120 mm, châssis hexagonal. Tout se règle dans `hexapod/config.py`.

## Architecture

| Module | Rôle |
| --- | --- |
| `hexapod/leg_ik.py` | IK et FK d'une patte 3 DDL (solution « genou en haut »). |
| `hexapod/body_ik.py` | IK du corps : translation (x, y, z) et rotation (roulis, tangage, lacet) avec pieds fixes au sol. |
| `hexapod/gait.py` | Démarche tripode : deux trépieds alternés, consigne `vx`, `vy`, `yaw_rate`, arrêt toujours pieds posés. |
| `hexapod/servo.py` | Conversion angle IK → angle servo (offset, sens, trim, butées) et pilote PCA9685. |
| `hexapod/robot.py` | Boucle de contrôle : démarche + pose du corps → IK → servos. |
| `sim/simulate.py` | Simulation 3D animée avec matplotlib. |

Repère du corps : x vers l'avant, y vers la gauche, z vers le haut.

## Utilisation

```python
from hexapod import Hexapod, GaitCommand, BodyPose, default_config
from hexapod.servo import PCA9685Driver

robot = Hexapod(default_config(), driver=PCA9685Driver())
robot.command = GaitCommand(vx=80.0, yaw_rate=0.2)  # mm/s, rad/s
robot.pose = BodyPose(z=10.0, pitch=0.05)           # mm, rad
while True:
    robot.update(0.02)                               # à appeler à 50 Hz
```

Sans `driver`, un pilote factice enregistre les angles au lieu de bouger les servos.

## Tests et simulation

```bash
python -m unittest -v                         # aucune dépendance
pip install matplotlib
python -m sim.simulate                        # marche avant
python -m sim.simulate --vx 0 --yaw 0.5       # rotation sur place
python -m sim.simulate --sway                 # marche + mouvements du corps
```

## Calibration

Chaque articulation a dans `config.py` un `zero_deg` (angle IK quand le servo est à 90°), un `direction` (±1 selon le montage) et un `trim_deg` à mesurer sur le robot. Par défaut, servo à 90° = coxa dans l'axe, fémur horizontal, tibia vertical.
