"""Version 1.1 du hexapode : corrections des pièces imprimées, sans tout refaire.

- body_top / body_bottom à 150 % (le corps imprimé), renforcés :
  * trous de fixation des pattes prêts pour inserts M2 (Ø3,0 mm),
  * 4 trous du support électronique prêts pour inserts M3 (Ø4,0 mm),
  * croisillons de body_bottom élargis.
- leg_top / thigh_top : les cache-vis qui dépassent de la plaque et frottent
  sur joint_top sont supprimés ; lamage pour que la tête de vis M2 soit à fleur.
  La fonction marche sur n'importe quelle version (100 % ou « 1.5x ») : donne-lui
  le STL réellement imprimé.

Dépendances : pip install manifold3d trimesh numpy shapely scipy
Usage : python cad/v1_1/generer_pieces_v1_1.py
"""

import os

import numpy as np
import trimesh
from manifold3d import Manifold, Mesh
from shapely.geometry import box as sbox
from shapely.ops import unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
PIECES = os.path.join(HERE, "..", "pieces_reelles")
OUT_DIR = os.path.join(HERE, "stl")
SEG = 48

# Inserts à chaud du kit : M2 Ø3,2 mm, M3 Ø4,2 mm -> trous 0,2 mm plus petits.
INSERT_M2_D = 3.0
INSERT_M3_D = 4.0
ELEC_HOLE_SPAN = (60.0, 51.0)  # entraxe des 4 trous du support électronique (body_top à 150 %)

# Cache-vis : tout ce qui dépasse de la plaque autour des trous de vis.
HEAD_D, HEAD_DEPTH = 4.2, 1.6  # lamage pour tête de vis M2 (bombée ou cylindrique basse)
BOSS_CLEAR_R = 5.0  # rayon (dans le plan) autour d'un trou où l'on rase les bossages

BRACE_GROW = 1.5  # élargissement des croisillons de body_bottom, par côté


def load(path):
    return trimesh.load(path)


def to_manifold(tm):
    tm = tm.copy()
    tm.fix_normals()  # certains STL d'origine ont des faces mal orientées
    mesh = Mesh(vert_properties=np.asarray(tm.vertices, dtype=np.float32),
                tri_verts=np.asarray(tm.faces, dtype=np.uint32))
    mesh.merge()
    m = Manifold(mesh)
    if m.status().name != "NoError":
        raise ValueError("maillage non manifold : {}".format(m.status()))
    return m


def to_trimesh(m):
    mesh = m.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3], faces=np.asarray(mesh.tri_verts))
    tm.merge_vertices()
    # Écarte les éclats plats (volume nul) que laisse parfois une découpe à fleur.
    parts = tm.split(only_watertight=False)
    if len(parts) > 1:
        tm = max(parts, key=lambda p: abs(p.volume) if p.is_watertight else 0)
    return tm


def section(tm, z):
    s = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    T = np.eye(4)
    T[2, 3] = -z
    p, _ = s.to_2D(to_2D=T)
    return p.polygons_full


def small_holes(tm, z, max_d=5.0):
    """Trous ronds de la coupe à la hauteur z : [(x, y, diamètre)]."""
    out = []
    for poly in section(tm, z):
        for ring in poly.interiors:
            area = trimesh.path.polygons.Polygon(ring).area
            d = 2 * np.sqrt(area / np.pi)
            if d <= max_d:
                c = ring.centroid
                out.append((c.x, c.y, d))
    return out


def plate_top(tm):
    """Hauteur du dessus de la plaque pleine (au-dessus, il ne reste que des bossages)."""
    lo, hi = tm.bounds[0][2], tm.bounds[1][2]
    full = sum(p.area for p in section(tm, lo + 0.05))
    z = lo + 0.05
    while z < hi:
        if sum(p.area for p in section(tm, z)) < 0.6 * full:
            return round(z - 0.05, 1)
        z += 0.05
    return hi


def cyl(d, h, x, y, z0):
    return Manifold.cylinder(h, d / 2, d / 2, SEG).translate([x, y, z0])


def raser_cache_vis(tm):
    """Supprime les bossages au-dessus de la plaque près des trous de vis et lamine les trous."""
    lo = tm.bounds[0][2]
    top = plate_top(tm)
    # Trous des vis M2 (Ø2,1) seulement : les autres petits trous ne portent pas de cache-vis.
    holes = [h for h in small_holes(tm, lo + 0.3, max_d=2.6) if h[2] > 1.95]
    m = to_manifold(tm)
    cut = None
    for x, y, _ in holes:
        c = cyl(2 * BOSS_CLEAR_R, 20, x, y, top - 0.02) + cyl(HEAD_D, HEAD_DEPTH + 0.01, x, y, top - HEAD_DEPTH)
        cut = c if cut is None else cut + c
    return to_trimesh(m - cut), top, len(holes)


def body_top_v11(tm):
    lo = tm.bounds[0][2]
    holes = small_holes(tm, lo + 0.3)
    m = to_manifold(tm)
    hi = tm.bounds[1][2]
    cx, cy = np.mean([h[0] for h in holes]), np.mean([h[1] for h in holes])
    n2 = n3 = 0
    for x, y, d in holes:
        elec = abs(abs(x - cx) - ELEC_HOLE_SPAN[0] / 2) < 1.0 and abs(abs(y - cy) - ELEC_HOLE_SPAN[1] / 2) < 1.0
        if elec:
            m = m - cyl(INSERT_M3_D, hi + 2, x, y, lo - 1)
            n3 += 1
        else:
            m = m - cyl(INSERT_M2_D, hi + 2, x, y, lo - 1)
            n2 += 1
    return to_trimesh(m), n2, n3


def body_bottom_v11(tm):
    """Élargit les croisillons : on épaissit la plaque vers l'intérieur des évidements centraux."""
    lo = tm.bounds[0][2]
    z_plate = lo + 0.3
    polys = section(tm, z_plate)
    plate = unary_union(polys)
    # Épaisseur de la plaque pleine (même méthode que plate_top, sur la coupe complète).
    top = plate_top(tm)
    minx, miny, maxx, maxy = plate.bounds
    # Zone des croisillons seulement : on ne touche ni au cadre ni aux encoches où
    # se clipsent les parois et la pièce centrale.
    inner = sbox(minx + 32, miny + 40, maxx - 32, maxy - 40)
    grown = plate.buffer(BRACE_GROW, join_style=2).intersection(inner)
    add = None
    geoms = getattr(grown, "geoms", [grown])
    for g in geoms:
        if g.is_empty or g.area < 1:
            continue
        ext = trimesh.creation.extrude_polygon(g, top - lo)
        ext.apply_translation([0, 0, lo])
        a = to_manifold(ext)
        add = a if add is None else add + a
    m = to_manifold(tm)
    if add is not None:
        m = m + add
    return to_trimesh(m), top


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    def save(name, tm):
        tm.export(os.path.join(OUT_DIR, name + ".stl"))
        e = tm.extents
        print("{:24s} {:6.1f} x {:6.1f} x {:5.1f} mm  étanche={}".format(name, e[0], e[1], e[2], tm.is_watertight))

    bt, n2, n3 = body_top_v11(load(os.path.join(PIECES, "body_top1.5x.stl")))
    print("body_top : {} trous pour inserts M2, {} pour inserts M3".format(n2, n3))
    save("body_top1.5x_v1.1", bt)
    bb, top = body_bottom_v11(load(os.path.join(PIECES, "body_bottom1.5x.stl")))
    print("body_bottom : plaque de {:.1f} mm, croisillons élargis de {} mm par côté".format(top, BRACE_GROW))
    save("body_bottom1.5x_v1.1", bb)
    for name in ("leg_top", "thigh_top"):
        src = os.path.join(PIECES, name + ".stl")
        if not os.path.isfile(src):
            continue
        tm, top, n = raser_cache_vis(load(src))
        print("{} : plaque jusqu'à z={:.1f}, {} trous lamés".format(name, top, n))
        save(name + "_v1.1", tm)


if __name__ == "__main__":
    main()
