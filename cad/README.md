# Pièces 3D (Fusion 360)

- `pieces_150/` : les 18 STL de [hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697/tree/master/mechanism)
  (SmallpTsai, licence GPL, voir `pieces_150/LICENSE`) déjà mis à l'échelle 150 %, en mm.
  Ils s'impriment tels quels, sans réglage d'échelle dans le slicer.
- `v2/` : pièces V2 (manchon servo, support électronique, verrou des prises, clips câbles) et les 10 pistes d'amélioration dans `v2/AMELIORATIONS.md`.
- `fusion360/ImportPiecesHexapode/` : script Fusion 360 qui crée un design avec une
  pièce par composant, rangées en deux groupes (Corps, Patte), avec la quantité à
  imprimer dans le nom (ex. `joint_top (x12)`).

## Importer dans Fusion 360

1. Garde `pieces_150/` et `fusion360/` côte à côte (comme dans ce dossier).
2. Fusion 360 : **Utilities > Add-Ins > Scripts and Add-Ins** (touche `Shift+S`).
3. Onglet **Scripts**, bouton **+**, choisis le dossier `fusion360/ImportPiecesHexapode`.
4. Sélectionne **ImportPiecesHexapode** puis **Run**.

Si le script ne trouve pas les STL, il demande le dossier `pieces_150`.

Les pièces arrivent en maillages. Pour les modifier (perçages, supports...) :
clic droit sur le corps > **Mesh > Modify > Convert Mesh**, méthode **Prismatic**.
Le design est en mode direct (sans historique) ; **Capture Design History** dans
le menu du composant racine repasse en mode paramétrique.

Le 150 % est uniforme ; les cotes mesurées sur le robot (coxa 45, fémur 59,
tibia 135 mm) diffèrent un peu du calcul (42 ; 63,9 ; 133,6).
