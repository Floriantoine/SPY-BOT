# SPY-BOT V2 : 10 pistes d'amélioration

## Le diagnostic d'abord

Les pièces de hexapod-v2-7697 ont été agrandies à 150 %, **mais pas les servos**.
Toutes les interfaces avec le servo sont donc fausses :

| Interface | Prévu (100 %) | Imprimé (150 %) | Servo MG92B réel |
|---|---|---|---|
| Logement du corps de servo | 23,8 x 13 mm | 35,7 x 19,5 mm | 22,8 x 12 mm |
| Plaque qui porte la bride | 3 mm | 4,5 mm | bride de 32,5 mm de long |
| Trous de vis | 2,1 mm (M2) | 3,15 mm | vis M2 |
| Logement hexagonal des articulations | 7,4 mm sur plats | 11,1 mm | — |

La bride du servo (32,5 mm) passe à travers le logement de 35,7 mm : le servo
n'est plus tenu par sa bride. C'est pour ça que tu dois visser les palonniers en
plastique sur les pièces imprimées. Et la charge est 2,4 kg·cm sur le servo de
fémur en marche tripode (robot de ~850 g, pied à 8,6 cm), soit ~78 % du MG92B.

## Les 10 pistes

Classées de la plus utile à la moins urgente. **(V2)** = pièce déjà dessinée dans `stl/`.

1. **Manchon de servo (V2)** : `manchon_servo.stl` x18. Il remplit le logement
   agrandi et ramène l'ouverture à la taille du MG92B, avec les avant-trous des
   vis de bride (M2 auto-taraudeuses). Le servo retrouve un appui franc sans palonnier.
2. **Vis M3 au lieu de M2** : à 150 % les trous font 3,15 mm, donc les vis
   d'assemblage d'origine deviennent des M3 (M2x30 → M3x45, M2x10 → M3x16).
   Avec des M2 dans des trous de 3 mm, tout prend du jeu.
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
   les 4 trous existants de `body_top` (entraxe 60 x 51 mm, vis M3
   auto-taraudeuses), entretoises pour les 2 PCA9685 (vis M2.5), berceau ESP32
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

- Mesure ton MG92B au pied à coulisse (corps et entraxe des trous de bride) et
  ajuste `SERVO_L`, `SERVO_W`, `SERVO_HOLE_PITCH` dans `generer_pieces_v2.py`.
- Imprime **un seul** manchon et essaie-le dans une pièce avant d'en faire 18.
- Mesure ta carte ESP32 (`ESP_L`, `ESP_W`) et l'entraxe des trous de tes
  PCA9685 (`PCA_HOLE_X`, `PCA_HOLE_Y`) : les clones varient.
- Mesure la hauteur du haut des prises servo branchées (`PLUG_H`).

Puis relance `python cad/v2/generer_pieces_v2.py` (dépendances :
`pip install manifold3d trimesh numpy`).
