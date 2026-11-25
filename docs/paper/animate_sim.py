#!/usr/bin/env python3
"""Side-by-side animation of one bump-model mission: v2 vs v3 vs v3.1.
  python animate_sim.py TRACE.json OUT.mp4"""
import json, sys, bisect
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
matplotlib.rcParams['animation.ffmpeg_path'] = '/opt/miniconda3/bin/ffmpeg'
D = json.load(open(sys.argv[1])); OUT = sys.argv[2]
C = dict(v2='#0072B2', v3i='#E69F00', v31='#009E73', red='#D55E00', glass='#56B4E9', wall='#555555', bar='#E69F00', hull='#F0C000')
names = [('v2', 'AHPPA v2 — no contact memory'), ('v3i', 'AHPPA v3 — contact memory'), ('v31', 'AHPPA v3.1 — memory + surface fusion')]
SPEED, FPS = 4.0, 30
T_END = max(D['runs'][k]['trace'][-1][0] for k, _ in names) + 3.0
frames = int(T_END / SPEED * FPS) + FPS * 3
plt.rcParams.update({'font.family': 'DejaVu Sans'})
fig, axs = plt.subplots(1, 3, figsize=(16, 6.4), dpi=100)
fig.patch.set_facecolor('white')
fig.suptitle('Same mission, a glass pane the sensors cannot see  (bump-model simulation, report Section 7.4)', fontsize=14, y=0.98)
W = np.array(D['walls']); G = np.array(D['glass']); sx, sy = D['start']; gx, gy = D['goal']
art = {}
for ax, (k, title) in zip(axs, names):
    ax.scatter(W[:, 0], W[:, 1], s=1.5, c=C['wall'], marker='s')
    ax.scatter(G[:, 0], G[:, 1], s=2, c=C['glass'], marker='s')
    ax.text(G[:, 0].mean() + 0.3, G[:, 1].min() + 0.3, 'glass\n(unseen)', color='#1f6f9f', fontsize=9)
    ax.plot(sx, sy, 's', ms=9, color='#222222'); ax.plot(gx, gy, '*', ms=16, color='#222222')
    ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])
    ax.set_xlim(W[:, 0].min() - 0.6, W[:, 0].max() + 0.6); ax.set_ylim(W[:, 1].min() - 1.6, W[:, 1].max() + 2.4)
    for s in ax.spines.values(): s.set_visible(False)
    ax.set_title(title, fontsize=12, color=C[k], fontweight='bold')
    trail, = ax.plot([], [], '-', lw=1.6, color=C[k], alpha=0.85)
    drone, = ax.plot([], [], 'o', ms=10, color=C[k], mec='white', mew=1.5, zorder=5)
    hits = ax.scatter([], [], s=40, marker='x', c=C['red'], zorder=4, linewidths=1.6)
    txt = ax.text(0.5, 0.02, '', transform=ax.transAxes, fontsize=13, ha='center', va='bottom', fontweight='bold', color='#333333')
    ban = ax.text(0.5, 0.92, '', transform=ax.transAxes, fontsize=17, ha='center', va='center', fontweight='bold',
                  bbox=dict(boxstyle='round', fc='white', ec='#888888', alpha=0.92))
    art[k] = dict(ax=ax, trail=trail, drone=drone, hits=hits, txt=txt, ban=ban, lines=[])
tlab = fig.text(0.01, 0.03, '', ha='left', fontsize=12, color='#444444')
fig.text(0.99, 0.03, 'red x = contact with the glass    orange = remembered barrier    yellow = measured hull (relaxed)',
         ha='right', fontsize=9, color='#666666')
fig.subplots_adjust(left=0.01, right=0.99, top=0.88, bottom=0.08, wspace=0.03)

def upto(lst, t):
    i = bisect.bisect_right([e[0] for e in lst], t)
    return lst[:i]

def update(f):
    t = f * SPEED / FPS
    tlab.set_text(f'simulated time {min(t, T_END):5.1f} s   (played {SPEED:.0f}× faster)')
    for k, _ in names:
        R, A = D['runs'][k], art[k]
        tr = upto(R['trace'], t)
        if tr:
            xy = np.array([[p[1], p[2]] for p in tr])
            A['trail'].set_data(xy[:, 0], xy[:, 1]); A['drone'].set_data([xy[-1, 0]], [xy[-1, 1]])
        hs = upto(R['hits'], t)
        A['hits'].set_offsets(np.array([[h[1], h[2]] for h in hs]) if hs else np.empty((0, 2)))
        for l in A['lines']: l.remove()
        A['lines'] = []
        mem = upto(R['memory'], t)
        if mem:
            kinds = upto(R['kinds'], t)
            kind = kinds[-1][1] if kinds else 'bar'
            segs, col = (mem[-1][2], C['hull']) if (kind == 'hull' and mem[-1][2]) else (mem[-1][1], C['bar'])
            for s in segs:
                if kind == 'dot':
                    mx, my = (s[0] + s[2]) / 2, (s[1] + s[3]) / 2
                    A['lines'] += A['ax'].plot([mx], [my], 's', ms=6, color=C['bar'])
                else:
                    A['lines'] += A['ax'].plot([s[0], s[2]], [s[1], s[3]], '-', lw=4, color=col, alpha=0.85, solid_capstyle='round')
        A['txt'].set_text(f"contacts: {len(hs)}")
        res = R['result']; done = t >= tr[-1][0] if tr else False
        if done and R['trace'] and t >= R['trace'][-1][0]:
            if res['outcome'] == 'success':
                A['ban'].set_text(f"REACHED the goal: {res['bumps']} contacts, {res['time_s']:.0f} s"); A['ban'].set_color(C['v31'])
            else:
                A['ban'].set_text(f"TIME OUT: {res['bumps']} contacts"); A['ban'].set_color(C['red'])
    return []

ani = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / FPS)
ani.save(OUT, writer=animation.FFMpegWriter(fps=FPS, codec='libx264', bitrate=4000, extra_args=['-pix_fmt', 'yuv420p']))
print('saved', OUT, frames, 'frames')
