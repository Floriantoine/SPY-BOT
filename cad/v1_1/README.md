# Version 1.1 : faire marcher le robot actuel

Corrections minimales, sans changer de servo ni réimprimer les pattes entières.
STL dans `stl/`, régénérés par `python cad/v1_1/generer_pieces_v1_1.py`.

| Pièce | Quantité | Ce qui change |
|---|---|---|
| `leg_top1.5x_v1.1` | 6 | Les 8 cache-vis qui dépassaient de 2,4 mm et frottaient sur `joint_top` sont supprimés ; lamage Ø4,2 x 1,6 mm pour que la tête de vis M2 soit à fleur. |
| `thigh_top1.5x_v1.1` | 6 | Même correction (2 trous). |
| `body_top1.5x_v1.1` | 1 | Corps à 150 %. 12 trous de fixation des pattes à Ø3,0 pour inserts M2 x4, 4 trous du support électronique à Ø4,0 pour inserts M3 x4. |
| `test_palonnier` | 1 | Plaque de test : 3 empreintes du palonnier 1 bras (jeu 0,1 / 0,2 / 0,3 mm, repérées par 1, 2 ou 3 encoches). |
| `body_bottom1.5x_v1.1` | 1 | Corps à 150 %. Plots de fixation des pattes à l'échelle de `thigh_bottom1.5x` : écart entre les deux plots d'une paire porté de 9,0 à 9,4 mm pour que la languette (9 mm) entre sans forcer, trous agrandis de Ø2,7 à Ø3,3 mm (vis M3), croisillons centraux élargis de 1,5 mm par côté. Cadre et encoches de clipsage inchangés. |

## À vérifier avant d'imprimer

- Le `body_bottom1.5X` imprimé avant (remix) gardait les plots à 100 % : écart de
  6 mm pour une languette de 9 mm, d'où le `thigh_bottom1.5x` qui ne rentrait pas.
  `body_bottom1.5x_v1.1` repart du corps agrandi à 150 % en entier, plots compris.
- Les vis M2 doivent avoir une tête basse (bombée ou cylindrique, 1,6 mm max) pour
  rester à fleur. Une tête creuse 6 pans standard (2 mm) dépasse encore de 0,4 mm.

## Réglages d'impression conseillés pour le corps

PETG si possible (PLA sinon), 4 périmètres, 40 % de remplissage gyroïde, couches
de 0,2 mm. C'est ce qui rend le corps solide, plus que la forme.
