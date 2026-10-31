# SPY-BOT V2 : 10 pistes d'amélioration

## Le point de départ

Ton robot mélange deux échelles (fichiers dans `../pieces_reelles/`) :
le coxa (`joint_*1.5x`) et le tibia (`foot_top/bottom/ground1.5x`) sont allongés
de 50 % en gardant les fixations d'origine, tout le reste est à 100 %, y compris
les pièces qui tiennent les servos. Les servos rentrent donc bien.

Avec un tibia de 135 mm, le servo de fémur porte environ 2,4 kg·cm en marche
tripode (robot de ~850 g, pied à 8,6 cm), soit ~78 % du MG92B : c'est la vraie
limite du robot.

## Les 10 pistes

Classées de la plus utile à la moins urgente. **(V2)** = pièce déjà dessinée dans `stl/`.

1. **Pièces de coxa et de tibia à revoir** : les versions « 1.5x » ont des trous
   de 3 mm là où l'original avait du M2 : vis M3, ou trous refaits en 2,1 mm.
2. **Vis et inserts** : inserts à chaud M2/M3 au lieu des écrous et des vis qui
   taraudent le plastique, pour pouvoir démonter sans abîmer les trous.
3. **Pivot opposé sur roulement** : de l'autre côté de l'axe du servo, un
   roulement 3x10x4 (623) ou MR63 avec une vis M3 en axe. Le servo ne porte plus
   l'effort de flexion de la patte, seulement le couple. C'est la plus grosse
   amélioration mécanique ; elle demande de redessiner les pièces d'articulation.
4. **Palonnier métal** : un palonnier rond en aluminium (vérifier le nombre de
   dents de ton MG92B, souvent 20 ou 21) vissé à la pièce par 4 vis M2 au lieu du
   palonnier plastique qui se déforme et prend du jeu.
5. **Tibia plus court ou pied rentré** : à 110 mm au lieu de 135 mm, ou avec le
   tibia vertical, le couple du fémur tombe sous 50 % du MG92B. Réglage logiciel
   (posture) d'abord, pièce plus courte ensuite si besoin.
6. **Support électronique (V2)** : `support_electronique.stl`. Plateau vissé sur
   les 4 trous existants de `body_top` (entraxe 60 x 51 mm à 150 %, à régler
   si ton corps est à 100 %), entretoises pour les 2 PCA9685 (vis M2.5), berceau ESP32
   tenu par 2 colliers, fentes de colliers pour les faisceaux.
7. **Verrou des prises servo (V2)** : `verrou_connecteurs.stl` x2. Une barrette
   qui appuie sur les 16 prises d'une PCA9685 et se visse par les 2 trous de la
   carte côté connecteurs. Plus de prise qui se débranche en marchant.
8. **Câbles tenus le long des pattes (V2)** : `clip_cables.stl` x12, vis M3
   dans les trous existants. Le câble ne tire plus sur la prise ni ne se coince
   dans une articulation. Laisser une boucle souple à chaque articulation.
9. **Distribution d'alimentation propre** : un bornier à vis (ou une petite carte
   à souder une fois) pour le 5-6 V des servos, des fils de 1 mm² minimum vers
   chaque PCA9685, un condensateur de 1000 µF sur chaque V+. Les connecteurs
   Dupont ne sont pas faits pour 5 A.
10. **Pied en TPU et allègement** : imprimer `foot_tip` en TPU (adhérence, chocs
    amortis), et les pièces de patte avec 2 périmètres et 15-20 % de remplissage
    en gyroïde pour gagner 50 à 80 g.

## Ce qu'il faut vérifier avant d'imprimer les pièces V2

- Vérifie l'échelle de ton `body_top` : trous de fixation à 60 x 51 mm (150 %)
  ou 40 x 34 mm (100 %), à régler dans `MOUNT_X` / `MOUNT_Y`.
- Mesure ta carte ESP32 (`ESP_L`, `ESP_W`) et l'entraxe des trous de tes
  PCA9685 (`PCA_HOLE_X`, `PCA_HOLE_Y`) : les clones varient.
- Mesure la hauteur du haut des prises servo branchées (`PLUG_H`).

Puis relance `python cad/v2/generer_pieces_v2.py` (dépendances :
`pip install manifold3d trimesh numpy`).
