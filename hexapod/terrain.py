"""Terrain accidenté de la simulation : ondulations, cailloux et trous.

Le robot n'a pas de capteur sous les pieds : il pose ses pieds sur le plan
qu'il croit être le sol. Sur ce terrain, il repose en vrai sur ses pieds
posés : le corps penche selon le terrain sous eux, et un pied au-dessus d'un
trou reste en l'air (ou un caillou soulève un coin du robot).

Hauteurs en mm, positions sur le sol en mm (repère de l'odométrie).
"""

import math

# Ondulations : (amplitude mm, longueur d'onde mm, direction rad, phase rad)
WAVES = ((15.0, 1100.0, 0.0, 0.0), (10.0, 800.0, 1.2, 1.0), (7.0, 520.0, 2.4, 2.0))
ROCK_COUNT = 160
ROCK_AREA = 2400.0  # mm : demi-côté du carré couvert de cailloux, autour du départ
ROCK_CLEAR = 220.0  # mm : rien sous le robot au départ


def _rocks(seed=7):
    """Cailloux (hauteur > 0) et trous (< 0) : (x, y, rayon, hauteur), toujours les mêmes."""
    state = seed

    def rnd():
        nonlocal state
        state = (state * 1103515245 + 12345) & 0x7FFFFFFF
        return state / 0x7FFFFFFF

    out = []
    while len(out) < ROCK_COUNT:
        x, y = (rnd() * 2 - 1) * ROCK_AREA, (rnd() * 2 - 1) * ROCK_AREA
        radius = 35.0 + 75.0 * rnd()
        height = -22.0 + 54.0 * rnd()  # trous jusqu'à 22 mm, cailloux jusqu'à 32 mm
        if x * x + y * y < (ROCK_CLEAR + radius) ** 2:
            continue
        out.append((round(x), round(y), round(radius), round(height)))
    return tuple(out)


ROCKS = _rocks()


def height(x, y):
    """Hauteur du terrain en (x, y)."""
    h = 0.0
    for a, wl, d, ph in WAVES:
        h += a * math.sin(2 * math.pi / wl * (math.cos(d) * x + math.sin(d) * y) + ph)
    for rx, ry, r, rh in ROCKS:
        dx, dy = x - rx, y - ry
        d2 = dx * dx + dy * dy
        if d2 < 9 * r * r:
            h += rh * math.exp(-d2 / (r * r))
    return h


def describe():
    """Le terrain en une ligne, pour que l'appli le dessine à l'identique."""
    w = " ".join("%g %g %g %g" % v for v in WAVES)
    r = " ".join("%d %d %d %d" % v for v in ROCKS)
    return "TERRAINMAP %d %s %d %s" % (len(WAVES), w, len(ROCKS), r)


def rest_on(feet, odom, swing=()):
    """Plan sur lequel le robot repose, d'après ses pieds posés.

    `feet` : pieds au sol, dans le repère du corps (mm), `odom` = (x, y, cap).
    Le z de chaque pied est sa hauteur par rapport au pied posé normal (un
    pied allongé de 10 mm : z = -10). `swing` : pieds en l'air ; s'il y en a
    un qui touche le terrain (caillou), le robot s'appuie aussi dessus.
    Retourne (a, b, c, roulis, tangage, en_l_air) : plan z = a + b·x + c·y sur
    le sol, assiette qu'il donne au robot (rad, repère corps), et pour chaque
    pied posé la hauteur dont il est au-dessus du terrain (0 = il touche).
    """
    x0, y0, th = odom
    c, s = math.cos(th), math.sin(th)

    def point(f):
        wx, wy = x0 + c * f[0] - s * f[1], y0 + s * f[0] + c * f[1]
        # Hauteur que ce pied impose au plan des pieds normaux.
        return (wx, wy, height(wx, wy) - f[2])

    pts = [point(f) for f in feet]
    if len(pts) < 3:
        h = height(x0, y0)
        return h, 0.0, 0.0, 0.0, 0.0, [0.0] * len(pts)
    support = _tripod(pts, x0, y0)
    if support:
        # Le robot repose sur les 3 pieds qui portent son centre ; les autres
        # sont au-dessus du terrain ou dans le vide.
        a, b, cc = support
        others = [point(f) for f in swing]
        if all(p[2] <= a + b * p[0] + cc * p[1] + 0.5 for p in others):
            return _result(a, b, cc, pts, c, s)
        pts_all = pts + others
        support = _tripod(pts_all, x0, y0)
        if support:
            return _result(support[0], support[1], support[2], pts, c, s)
    a, b, cc = _fit(pts)
    # Un pied en l'air qui rentrerait dans le terrain le touche : le robot
    # bascule aussi sur lui (on l'ajoute, au plus 3 fois).
    others = [point(f) for f in swing]
    support = list(pts)
    for _ in range(3):
        worst, below = None, 0.5
        for p in others:
            d = p[2] - (a + b * p[0] + cc * p[1])
            if d > below:
                worst, below = p, d
        if worst is None:
            break
        others.remove(worst)
        support.append(worst)
        a, b, cc = _fit(support)
    # Aucun pied ne s'enfonce : le robot monte jusqu'à toucher le plus haut.
    a += max(p[2] - (a + b * p[0] + cc * p[1]) for p in support)
    return _result(a, b, cc, pts, c, s)


def _result(a, b, cc, pts, c, s):
    air = [a + b * p[0] + cc * p[1] - p[2] for p in pts]
    ahead, left = c * b + s * cc, -s * b + c * cc
    # Sol qui monte devant : nez vers le haut (tangage négatif) ;
    # sol qui monte à gauche : côté gauche plus haut (roulis positif).
    return a, b, cc, math.atan(left), -math.atan(ahead), air


def _tripod(pts, cx, cy):
    """Plan des 3 pieds sur lesquels le robot repose vraiment : aucun autre
    pied ne s'enfonce et le centre (cx, cy) est au-dessus de leur triangle
    (face du dessus de l'enveloppe des points). None si aucun ne convient."""
    n = len(pts)
    best = None
    for i in range(n - 2):
        for j in range(i + 1, n - 1):
            for k in range(j + 1, n):
                p, q, r = pts[i], pts[j], pts[k]
                # Centre dans le triangle (coordonnées barycentriques).
                d = (q[1] - r[1]) * (p[0] - r[0]) + (r[0] - q[0]) * (p[1] - r[1])
                if abs(d) < 1e-6:
                    continue
                u = ((q[1] - r[1]) * (cx - r[0]) + (r[0] - q[0]) * (cy - r[1])) / d
                v = ((r[1] - p[1]) * (cx - r[0]) + (p[0] - r[0]) * (cy - r[1])) / d
                if u < -1e-9 or v < -1e-9 or u + v > 1 + 1e-9:
                    continue
                a, b, cc = _fit((p, q, r))
                over = max(t[2] - (a + b * t[0] + cc * t[1]) for t in pts)
                if over <= 1e-6:
                    return a, b, cc
                if best is None or over < best[0]:
                    best = (over, a, b, cc)
    # Arrondis : le moins mauvais, remonté pour que rien ne s'enfonce.
    if best is not None and best[0] < 0.5:
        return best[1] + best[0], best[2], best[3]
    return None


def _fit(pts):
    """Plan z = a + b·x + c·y des moindres carrés (exact avec 3 points)."""
    # Moindres carrés (exact avec 3 pieds), coordonnées centrées.
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    mz = sum(p[2] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    syy = sum((p[1] - my) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    sxz = sum((p[0] - mx) * (p[2] - mz) for p in pts)
    syz = sum((p[1] - my) * (p[2] - mz) for p in pts)
    det = sxx * syy - sxy * sxy
    if abs(det) < 1e-6:
        b = cc = 0.0
    else:
        b = (sxz * syy - syz * sxy) / det
        cc = (syz * sxx - sxz * sxy) / det
    return mz - b * mx - cc * my, b, cc
