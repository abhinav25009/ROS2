#!/usr/bin/env python3
"""Top-down maps of the Gazebo worlds, drawn from the SDF collision geometry.
  python world_maps.py WORLD_DIR OUT_DIR"""
import math, os, sys
import xml.etree.ElementTree as ET
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

WD, OUT = sys.argv[1], sys.argv[2]


def pose(el):
    p = el.find('pose') if el is not None else None
    v = [float(t) for t in p.text.split()] if p is not None and p.text else []
    return (v + [0] * 6)[:6]


def compose(a, b):              # yaw-only composition is enough for these worlds
    x, y, z, _, _, ya = a
    bx, by, bz, br, bp, byaw = b
    c, s = math.cos(ya), math.sin(ya)
    return (x + c * bx - s * by, y + s * bx + c * by, z + bz, br, bp, ya + byaw)


def shapes(path):
    root = ET.parse(path).getroot()
    out = []
    for m in root.iter('model'):
        mp = pose(m)
        for l in m.findall('link'):
            lp = compose(mp, pose(l))
            for c in l.findall('collision'):
                cp = compose(lp, pose(c))
                g = c.find('geometry')
                if g is None:
                    continue
                b, cy = g.find('box'), g.find('cylinder')
                if b is not None:
                    sx, sy, sz = (float(t) for t in b.find('size').text.split())
                    out.append(('box', m.get('name'), cp, (sx, sy, sz)))
                elif cy is not None:
                    out.append(('cyl', m.get('name'), cp, (float(cy.find('radius').text), float(cy.find('length').text))))
    return out


def draw(ax, sh, zlo, zhi, glass=()):
    for kind, name, (x, y, z, r, p, ya), dims in sh:
        if kind == 'box':
            sx, sy, sz = dims
            if abs(r) > 0.3 or abs(p) > 0.3:           # pitched roof slabs: skip in plan view
                continue
            z0, z1 = z - sz / 2, z + sz / 2
        else:
            rad, ln = dims
            z0, z1 = z - ln / 2, z + ln / 2
        if z1 < 0.15 or z1 < zlo or z0 > zhi:
            continue
        if kind == 'box' and sz < 0.35 and sx * sy > 2.0:  # floor / ceiling slabs
            continue
        if kind == 'box' and name in glass:                 # draw thin glass visibly
            sx = max(sx, 0.15)
        tall = z1 >= zlo + 1.0 and z0 <= zlo + 1.9      # blocks the flight band of this floor
        col = '#56B4E9' if name in glass else ('#3a3a3a' if tall else '#b8b8b8')
        if kind == 'box':
            c, s = math.cos(ya), math.sin(ya)
            pts = [(x + c * dx - s * dy, y + s * dx + c * dy) for dx, dy in
                   ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]
            ax.add_patch(Polygon(pts, closed=True, fc=col, ec='none', alpha=0.9 if name not in glass else 0.8))
        else:
            ax.add_patch(Circle((x, y), max(dims[0], 0.04), fc=col, ec='none'))
    ax.set_aspect('equal')
    ax.autoscale_view()
    if any(n in glass for _, n, _, _ in sh):
        ax.annotate('glass pane\n(collision only:\ninvisible to cameras)', (4.1, 0.8), xytext=(5.0, 1.6),
                    fontsize=7, color='#1f6f9f', arrowprops=dict(arrowstyle='->', color='#1f6f9f'))
        ax.annotate('gap', (4.0, 5.6), fontsize=7, color='#666666')
    ax.grid(color='#eeeeee', lw=0.5)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    ax.set_xlabel('x (m)', fontsize=8)
    ax.tick_params(labelsize=7)


plt.rcParams.update({'font.family': 'DejaVu Sans'})
W = [('house_3room.sdf', 'house_3room — the first world (single floor)', [(0, 2.9, 'flight band 1.0–2.6 m')]),
     ('indoor_10x8x3.sdf', 'indoor_10x8x3 — small test room', [(0, 2.9, 'flight band')]),
     ('house_2floor.sdf', 'house_2floor — two floors, atrium, railing', [(0, 2.85, 'ground floor'), (2.9, 7.5, 'first floor')]),
     ('glass_test.sdf', 'glass_test — unseen glass pane', [(0, 2.8, 'flight band')])]
for f, title, floors in W:
    sh = shapes(os.path.join(WD, f))
    fig, axs = plt.subplots(1, len(floors), figsize=(3.4 * len(floors), 3.0), squeeze=False)
    for ax, (lo, hi, lab) in zip(axs[0], floors):
        draw(ax, sh, lo, hi, glass=('glass_pane',))
        ax.set_title(lab, fontsize=8, color='#444444')
    axs[0][0].set_ylabel('y (m)', fontsize=8)
    fig.suptitle(title, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'world_' + f.replace('.sdf', '') + '.png'), dpi=200, facecolor='white')
    plt.close(fig)
    print(f, len(sh), 'shapes')
