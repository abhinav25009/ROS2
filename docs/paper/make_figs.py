#!/usr/bin/env python3
"""Figures for the AHPPA v3 report and paper draft.

  python make_figs.py DATA_DIR
DATA_DIR holds bump_summary.json, px4_flights.json (optional) and the
flight logs used for the trajectory figure (optional).
"""
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

D = sys.argv[1]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'figs')
os.makedirs(OUT, exist_ok=True)

# Okabe-Ito (colour-blind safe), fixed order: identity follows the planner
C = dict(base='#999999', v2='#0072B2', v3='#E69F00', v31='#009E73', ink='#222222', mute='#666666',
         glass='#56B4E9', crash='#D55E00')
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8.5, 'axes.spines.top': False,
                     'axes.spines.right': False, 'axes.edgecolor': '#888888', 'axes.labelcolor': C['ink'],
                     'xtick.color': C['mute'], 'ytick.color': C['mute'], 'axes.grid': True,
                     'grid.color': '#e6e6e6', 'grid.linewidth': 0.6, 'axes.axisbelow': True,
                     'legend.frameon': False, 'savefig.bbox': 'tight', 'savefig.dpi': 200})


def save(fig, name):
    for ext in ('pdf', 'png'):
        fig.savefig(os.path.join(OUT, f'{name}.{ext}'), facecolor='white')
    plt.close(fig)
    from PIL import Image                      # flatten: an alpha channel prints black in Chrome's PDF
    f = os.path.join(OUT, f'{name}.png')
    Image.open(f).convert('RGB').save(f)


# ── 1. static planning (1,320 queries) ──────────────────────────────────────
static = [('Stock A*', 0.00, 707, 'base'), ('AHPPA (shipped)', -2.65, 1372, 'base'),
          ('AHPPA, MEDIUM→LT*', -3.79, 1886, 'base'), ('Lazy Theta*', -4.01, 2013, 'base'),
          ('AHPPA v2 (A*+smoothing)', -3.80, 723, 'v2')]
fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.2))
y = np.arange(len(static))[::-1]
for i, (n, L, T, k) in enumerate(static):
    ax[0].barh(y[i], L, color=C[k], height=0.6)
    ax[0].text(L - 0.08, y[i], f'{L:+.2f}%', va='center', ha='right', color=C['ink'], fontsize=7.5)
    ax[1].barh(y[i], T, color=C[k], height=0.6)
    ax[1].text(T + 30, y[i], f'{T:,} s', va='center', ha='left', color=C['ink'], fontsize=7.5)
ax[0].set_yticks(y, [s[0] for s in static])
ax[1].set_yticks(y, [''] * len(y))
ax[0].set_xlim(-5.4, 0.3)
ax[0].set_xlabel('mean path length vs stock A*')
ax[1].set_xlim(0, 2500)
ax[1].set_xlabel('total planning time, 1,320 queries')
for a in ax:
    a.grid(axis='y', visible=False)
save(fig, 'fig_static')

# ── 2. bump-model simulation ────────────────────────────────────────────────
B = json.load(open(os.path.join(D, 'bump_summary.json')))
sc = ['railing', 'netting', 'post_door', 'glass_corr', 'glass', 'zigzag']
lab = dict(railing='railing', netting='netting', post_door='door\npost',
           glass_corr='glass\ndoor', glass='glass\npane', zigzag='two\npanes')
pl = [('v2', 'v2 (and stock, LT*, AHPPA)', 'v2'), ('v3i', 'v3 as flown', 'v3'), ('v31', 'v3.1 (fusion)', 'v31')]
fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.4))
x = np.arange(len(sc))
w = 0.26
for j, (k, n, c) in enumerate(pl):
    b = [B[s][k]['bumps'] for s in sc]
    e = [B[s][k]['exp'] for s in sc]
    ax[0].bar(x + (j - 1) * w, b, w * 0.92, color=C[c], label=n)
    ax[1].bar(x + (j - 1) * w, e, w * 0.92, color=C[c], label=n)
ax[0].set_yscale('log')
ax[0].set_ylim(0.5, 150)
ax[0].set_ylabel('contacts per mission (log)')
ax[1].set_ylabel('P(survive all contacts)\nreach × (1−0.48)^contacts')
ax[1].set_ylim(0, 0.65)
for a in ax:
    a.set_xticks(x, [lab[s] for s in sc], fontsize=7)
    a.grid(axis='x', visible=False)
ax[0].legend(loc='upper left', fontsize=7, ncol=1)
save(fig, 'fig_bump')

# ── 3. barrier fusion schematic (contacts of PX4 flight B20-v3-10) ─────────
sys.path.insert(0, D)
import portfolio_geom as G  # noqa: E402  (memory_segments copied from portfolio.py)
hits = [(3.55, 1.40, 0.94, 0.35), (3.61, -1.26, 0.84, -0.54), (3.58, 4.16, 0.99, -0.12)]
mem = []
for x0, y0, hx, hy in hits:
    n = math.hypot(hx, hy)
    mem.append(dict(cx=x0 + hx / n * 0.3, cy=y0 + hy / n * 0.3, hx=hx / n, hy=hy / n))
fig, ax = plt.subplots(1, 3, figsize=(7.0, 2.6), sharey=True)
titles = ['v3: one bar per contact,\nacross the heading', 'v3.1: contacts fused\ninto one surface (bar)',
          'v3.1 relaxed: measured\nhull (when bars seal the gap)']
cfg = [('bar', False), ('bar', True), ('hull', True)]
for a, t, (kind, fuse) in zip(ax, titles, cfg):
    a.add_patch(plt.Rectangle((-3, -3), 14, 10, fill=False, ec='#444444', lw=1.2))
    a.plot([4, 4], [-2.8, 4.5], color=C['glass'], lw=3, solid_capstyle='butt', label='unseen glass')
    a.annotate('gap', (4.15, 5.6), color=C['mute'], fontsize=7)
    for s in G.memory_segments(mem, kind, fuse):
        a.plot([s[0], s[2]], [s[1], s[3]], color=C['v31'] if fuse else C['v3'], lw=2.2)
    for x0, y0, hx, hy in hits:
        a.plot(x0, y0, 'o', ms=4, color=C['crash'])
        a.annotate('', (x0 + hx * 0.6, y0 + hy * 0.6), (x0 - hx * 0.6, y0 - hy * 0.6),
                   arrowprops=dict(arrowstyle='->', color=C['crash'], lw=0.9))
    a.set_title(t, fontsize=8, color=C['ink'])
    a.set_xlim(-0.5, 8.5)
    a.set_ylim(-3.3, 7.3)
    a.set_aspect('equal')
    a.set_xlabel('x (m)')
    a.grid(False)
ax[0].set_ylabel('y (m)')
save(fig, 'fig_fusion')

# ── 4. PX4 outcomes ─────────────────────────────────────────────────────────
pf = os.path.join(D, 'px4_flights.json')
if os.path.exists(pf):
    F = json.load(open(pf))
    order = [('v2', 'v2'), ('v3', 'v3 (as flown)'), ('v31', 'v3.1')]
    fig, ax = plt.subplots(1, 2, figsize=(7.0, 2.0))
    for i, (k, n) in enumerate(order):
        fl = [f for f in F if f['variant'] == k]
        if not fl:
            continue
        cnt = [sum(f['outcome'] in o for f in fl) for o in (('REACHED',), ('timeout', 'gave_up'), ('CRASH',))]
        left = 0
        for v, o, c in zip(cnt, ('reached', 'gave up the goal', 'crashed'),
                           (C['v31'], '#CCCCCC', C['crash'])):
            ax[0].barh(i, v, left=left, color=c, height=0.6, label=o if i == 0 else None,
                       edgecolor='white', linewidth=1)
            if v:
                ax[0].text(left + v / 2, i, str(v), ha='center', va='center', fontsize=7.5,
                           color='white' if c != '#CCCCCC' else C['ink'])
            left += v
        ax[0].text(left + 0.3, i, f'n={len(fl)}', va='center', fontsize=7, color=C['mute'])
        con = [f['contacts'] for f in fl]
        ax[1].scatter(np.array(con) + np.random.default_rng(i).uniform(-0.15, 0.15, len(con)),
                      np.full(len(con), i) + np.random.default_rng(i + 9).uniform(-0.2, 0.2, len(con)),
                      s=12, color=C[k], edgecolor='white', linewidth=0.4)
    ax[0].set_yticks(range(len(order)), [n for _, n in order])
    ax[1].set_yticks(range(len(order)), [''] * len(order))
    ax[0].set_xlabel('flights')
    ax[1].set_xlabel('glass contacts per flight')
    ax[0].legend(loc='upper center', bbox_to_anchor=(0.5, -0.38), ncol=3, fontsize=7)
    for a in ax:
        a.grid(axis='y', visible=False)
    save(fig, 'fig_px4')
print('figures ->', OUT)

# ── 5. PX4 trajectories (one flight per variant, chosen in TRAJ) ────────────
TRAJ = os.environ.get('TRAJ', '')          # "v2=DIR:GY,v3=DIR:GY,v31=DIR:GY" under $LOGS
LOGS = os.environ.get('LOGS', '')
if TRAJ and LOGS:
    import csv
    sel = dict(kv.split('=') for kv in TRAJ.split(','))
    names = dict(v2='v2: bounces, crashes', v3='v3: re-hits the pane, crashes', v31='v3.1: one surface, through the gap')
    fig, ax = plt.subplots(1, len(sel), figsize=(7.0, 2.7), sharey=True)
    for a, (k, dg) in zip(np.atleast_1d(ax), sel.items()):
        d, gy = (dg.split(':') + ['0'])[:2]
        P = [(float(r['x']), float(r['y']), float(r['z']), float(r['tilt_deg']))
             for r in csv.DictReader(open(os.path.join(LOGS, d, 'pose.csv')))]
        P = np.array(P)
        P = P[np.argmax(P[:, 2] > 0.5):]                     # from take-off
        near = np.hypot(P[:, 0] - 8, P[:, 1] - float(gy)) < 0.6
        air = np.nonzero(P[:, 2] > 0.3)[0]
        end = (np.argmax(near) + 1) if near.any() else air[-1] + 1   # until reached, or last airborne sample
        P = P[:end]
        nav = open(os.path.join(LOGS, d, 'nav.log'), errors='ignore').read()
        lost = 'CRASH:' in nav
        on, calm = [], 0                                      # contact onsets (same rule as analysis)
        for i, a_ in enumerate(P[:, 3]):
            if a_ >= 10 and calm >= 5:
                on.append(i)
            calm = calm + 1 if a_ < 5 else 0
        a.add_patch(plt.Rectangle((-3, -3), 14, 10, fill=False, ec='#444444', lw=1.2))
        a.plot([4, 4], [-2.8, 4.5], color=C['glass'], lw=3, solid_capstyle='butt')
        a.plot(P[:, 0], P[:, 1], color=C[k], lw=1.1)
        a.plot(P[on, 0], P[on, 1], 'o', ms=4, mfc='none', mew=1.1, color=C['crash'])
        if lost:
            a.plot(P[-1, 0], P[-1, 1], 'x', ms=7, mew=1.6, color=C['crash'])
        a.plot(0, 0, 's', ms=4, color=C['ink'])
        a.plot(8, float(gy), '*', ms=7, color=C['ink'])
        a.set_title(names[k], fontsize=8, color=C['ink'])
        a.set_xlim(-1, 9.5)
        a.set_ylim(-3.3, 7.3)
        a.set_aspect('equal')
        a.set_xlabel('x (m)')
        a.grid(False)
    np.atleast_1d(ax)[0].set_ylabel('y (m)')
    save(fig, 'fig_traj')
    print('trajectory figure done')
