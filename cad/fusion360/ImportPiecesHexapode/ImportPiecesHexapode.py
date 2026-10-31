"""Importe les pièces réellement imprimées du hexapode dans Fusion 360.

Toutes les pièces en version « 1.5x » quand elle existe ; foot_tip et
tools_mount à 100 % comme hexapod-v2-7697.

Crée un nouveau design avec un composant par pièce, rangés en deux groupes
(Corps, Patte) et posés côte à côte sur le plan XY. Chaque pièce est importée
comme corps maillé (STL) ; pour la modifier, clic droit sur le corps >
« Mesh > Modify > Convert Mesh » (méthode « Prismatic »).

Utilisation : Utilities > Add-Ins > Scripts and Add-Ins > + (Scripts) >
choisir ce dossier, puis Run.
"""

import os
import traceback

import adsk.core
import adsk.fusion

# Dossier des STL : par défaut cad/pieces_reelles du dépôt (deux niveaux au-dessus).
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STL_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "pieces_reelles"))

# Pièces et quantités à imprimer (BOM de hexapod-v2-7697).
GROUPS = [
    ("Corps", [
        ("body_top1.5x", 1),
        ("body_bottom1.5x", 1),
        ("body_center1.5x", 1),
        ("body_side_wall1.5x", 2),
        ("body_back_wall1.5x", 1),
        ("battery_lock1.5x", 1),
        ("tools_mount", 1),
    ]),
    ("Patte", [
        ("thigh_top1.5x", 6),
        ("thigh_bottom1.5x", 6),
        ("joint_cross1.5x", 6),
        ("joint_top1.5x", 12),
        ("joint_bottom1.5x", 12),
        ("leg_top1.5x", 6),
        ("leg_bottom1.5x", 6),
        ("foot_top1.5x", 6),
        ("foot_bottom1.5x", 6),
        ("foot_ground1.5x", 6),
        ("foot_tip", 6),
    ]),
]

GAP_CM = 1.0  # écart entre deux pièces (unités de l'API : cm)
ROW_WIDTH_CM = 45.0  # largeur max d'une rangée avant de passer à la suivante


def ask_stl_dir(ui):
    dlg = ui.createFolderDialog()
    dlg.title = "Dossier contenant les STL du hexapode (pieces_reelles)"
    if dlg.showDialog() != adsk.core.DialogResults.DialogOK:
        return None
    return dlg.folder


def translate(occ, x, y, z):
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(x, y, z)
    occ.transform = m


def run(context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        stl_dir = DEFAULT_STL_DIR
        if not os.path.isfile(os.path.join(stl_dir, "body_top.stl")):
            stl_dir = ask_stl_dir(ui)
            if not stl_dir:
                return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        # Mode direct : pas d'historique, on peut placer librement les maillages.
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent

        missing = []
        y_cursor = 0.0
        for group_name, parts in GROUPS:
            group_occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            group_occ.component.name = group_name
            x, row_depth = 0.0, 0.0
            for name, qty in parts:
                path = os.path.join(stl_dir, name + ".stl")
                if not os.path.isfile(path):
                    missing.append(name)
                    continue
                occ = group_occ.component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                comp = occ.component
                comp.name = "{} (x{})".format(name, qty)
                bodies = comp.meshBodies.add(path, adsk.fusion.MeshUnits.MillimeterMeshUnit)
                body = bodies.item(0)
                body.name = name
                box = body.boundingBox
                w = box.maxPoint.x - box.minPoint.x
                d = box.maxPoint.y - box.minPoint.y
                if x > 0 and x + w > ROW_WIDTH_CM:
                    x = 0.0
                    y_cursor += row_depth + GAP_CM
                    row_depth = 0.0
                # Coin bas-gauche de la pièce en (x, y_cursor), posée sur z = 0.
                translate(occ, x - box.minPoint.x, y_cursor - box.minPoint.y, -box.minPoint.z)
                x += w + GAP_CM
                row_depth = max(row_depth, d)
            y_cursor += row_depth + 3 * GAP_CM

        app.activeViewport.fit()
        msg = "Pièces importées depuis :\n{}".format(stl_dir)
        if missing:
            msg += "\n\nIntrouvables : " + ", ".join(missing)
        ui.messageBox(msg, "SPY-BOT")
    except Exception:
        ui.messageBox("Erreur :\n{}".format(traceback.format_exc()), "SPY-BOT")
