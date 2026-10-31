# Pièces 3D (Fusion 360)

- `pieces_reelles/` : les pièces réellement imprimées, en mm, d'après
  [hexapod-v2-7697](https://github.com/SmallpTsai/hexapod-v2-7697/tree/master/mechanism)
  (SmallpTsai, licence GPL, voir `pieces_reelles/LICENSE`). Corps (`body_*1.5x`, simple
  agrandissement à 150 %) et remix « 1.5x » de toutes les pièces de patte (`joint_*`,
  `thigh_*`, `leg_*`, `foot_top/bottom/ground`) ; seuls `foot_tip` et `tools_mount`
  restent à 100 %.
- `v1_1/` : corrections pour faire marcher le robot actuel (voir `v1_1/README.md`).
- `v2/` : pièces V2 (support électronique, verrou des prises, clips câbles) et les 10 pistes d'amélioration dans `v2/AMELIORATIONS.md`.
- `fusion360/ImportPiecesHexapode/` : script Fusion 360 qui crée un design avec une
  pièce par composant, rangées en deux groupes (Corps, Patte), avec la quantité à
  imprimer dans le nom (ex. `joint_top (x12)`).

## Importer dans Fusion 360

1. Garde `pieces_reelles/` et `fusion360/` côte à côte (comme dans ce dossier).
2. Fusion 360 : **Utilities > Add-Ins > Scripts and Add-Ins** (touche `Shift+S`).
3. Onglet **Scripts**, bouton **+**, choisis le dossier `fusion360/ImportPiecesHexapode`.
4. Sélectionne **ImportPiecesHexapode** puis **Run**.

Si le script ne trouve pas les STL, il demande le dossier `pieces_reelles`.

Les pièces arrivent en maillages. Pour les modifier (perçages, supports...) :
clic droit sur le corps > **Mesh > Modify > Convert Mesh**, méthode **Prismatic**.
Le design est en mode direct (sans historique) ; **Capture Design History** dans
le menu du composant racine repasse en mode paramétrique.
