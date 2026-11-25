#!/usr/bin/env python3
"""Extra figures for the full-story report.  python story_plots.py VM_BENCH_DIR OUT_DIR"""
import csv, json, os, sys, collections
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
B, OUT = sys.argv[1], sys.argv[2]
C = dict(base='#999999', v2='#0072B2', v3='#E69F00', v31='#009E73', ink='#222222', mute='#666666', red='#D55E00', sky='#56B4E9', pink='#CC79A7')
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8.5, 'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.edgecolor': '#888888', 'xtick.color': C['mute'], 'ytick.color': C['mute'], 'axes.grid': True,
                     'grid.color': '#e6e6e6', 'grid.linewidth': 0.6, 'axes.axisbelow': True, 'legend.frameon': False,
                     'savefig.bbox': 'tight', 'savefig.dpi': 200})
def save(fig, n):
    fig.savefig(os.path.join(OUT, n + '.png'), facecolor='white'); plt.close(fig)

# 1. the original six-algorithm comparison in house_3room
R = list(csv.DictReader(open(os.path.join(B, 'benchmark_results.csv'))))
algs = ['A*', 'Dijkstra', 'Bellman Ford', 'Theta*', 'PRM', 'RRT']
st = {}
for a in algs:
    L = [r for r in R if r['Algorithm'] == a]; ok = [r for r in L if r['Success'] == 'True']
    m = lambda k: np.mean([float(r[k]) for r in ok])
    st[a] = dict(ok=len(ok), n=len(L), t=m('Time(ms)'), wp=m('Waypoints'), sm=m('Smoothness(deg)'), nodes=m('Nodes Explored'))
fig, ax = plt.subplots(1, 4, figsize=(7.2, 2.2))
cols = ['#0072B2', '#56B4E9', '#999999', '#009E73', '#E69F00', '#D55E00']
y = np.arange(len(algs))[::-1]
for k, (key, lab, log) in enumerate([('ok', 'test cases solved (of 5)', False), ('t', 'planning time (ms, log)', True),
                                     ('nodes', 'nodes explored (log)', True), ('sm', 'smoothness metric (deg,\nlower = smoother)', False)]):
    v = [st[a][key] for a in algs]
    ax[k].barh(y, v, color=cols, height=0.62)
    if log:
        ax[k].set_xscale('log')
        ax[k].xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        ax[k].set_xlim(right=max(v) * 6)
    ax[k].set_xlabel(lab, fontsize=7.5)
    ax[k].set_yticks(y, algs if k == 0 else [''] * len(algs))
    ax[k].grid(axis='y', visible=False)
    for yi, vi in zip(y, v):
        ax[k].text(vi, yi, f' {vi:.0f}' if key != 'ok' else f' {vi}', va='center', fontsize=6.5, color=C['ink'])
fig.suptitle('Before AHPPA: six planners on house_3room (5 test routes)', fontsize=9)
save(fig, 'base_six_planners')

# 2. full-mission evaluation (440 missions x 5 planners)
M = json.load(open(os.path.join(B, 'missions_results.json')))
pl = [('stock', 'Stock A*'), ('ps', 'A* + smoothing'), ('lt', 'Lazy Theta*'), ('ahppa', 'AHPPA (shipped)'), ('v2', 'AHPPA v2')]
fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.2), gridspec_kw=dict(width_ratios=[1, 1.6]))
o = M['outcomes']
yy = np.arange(len(pl))[::-1]
ax[0].barh(yy, [o[k]['success'] * 100 for k, _ in pl], color=[C['base'], C['v2'], C['sky'], C['base'], C['v2']], height=0.6)
for yi, (k, _) in zip(yy, pl):
    ax[0].text(o[k]['success'] * 100, yi, f" {o[k]['success']*100:.1f}%  (trips {o[k]['trips']})", va='center', fontsize=6.8)
ax[0].set_yticks(yy, [n for _, n in pl]); ax[0].set_xlim(80, 103); ax[0].set_xlabel('missions completed (%)')
ax[0].grid(axis='y', visible=False)
fam = [f for f in M['family'] if min(M['family'][f]) < 1.0]
x = np.arange(len(fam)); w = 0.16
for j, ((k, n), c) in enumerate(zip(pl, [C['base'], C['v2'], C['sky'], '#555555', C['v31']])):
    ax[1].bar(x + (j - 2) * w, [M['family'][f][j] * 100 for f in fam], w * 0.95, color=c, label=n)
ax[1].set_xticks(x, fam); ax[1].set_ylim(40, 102); ax[1].set_ylabel('completed (%)')
ax[1].legend(fontsize=6.5, ncol=3, loc='upper center', bbox_to_anchor=(0.5, -0.12)); ax[1].grid(axis='x', visible=False)
ax[1].set_title('layout families where any mission failed\n(every other family: 100% for all planners)', fontsize=7.5, color=C['mute'])
save(fig, 'missions')

# 3. project timeline
ev = [('Phase 1 (Jun)', 'Base stack:\nPX4, Gazebo,\nOctoMap, A*;\n6 planners\ncompared'),
      ('Phases 2–3', 'Frontier\nexploration,\n2.5D layers,\n360° rig,\nhouse_2floor'),
      ('25 Oct', 'AHPPA\nportfolio:\nclearance\nrule'),
      ('17 Nov', 'House\nbenchmark,\nfirst flights,\n44 layouts,\nceiling fix'),
      ('19 Nov', 'Baselines,\noracle, race:\nswitching\ndoes not pay'),
      ('19–20 Nov', 'v2 smoothing,\n2,200 missions,\nglass flights:\ncontact = bump,\nv3 memory'),
      ('21 Nov', 'v3.1 fusion,\n50 flights,\nharness bug\nfixed'),
      ('22–25 Nov', '60 more\nflights,\ndemo overlay,\nreport, paper')]
fig, ax = plt.subplots(figsize=(7.2, 2.0))
ax.axhline(0, color='#888888', lw=1)
for i, (d, t) in enumerate(ev):
    ax.plot(i, 0, 'o', color=C['v31'] if i >= 5 else (C['v2'] if i >= 2 else C['base']), ms=8, zorder=3)
    ax.text(i, 0.18, d, ha='center', va='bottom', fontsize=7.2, color=C['ink'], weight='bold')
    ax.text(i, -0.18, t, ha='center', va='top', fontsize=6.4, color=C['ink'], linespacing=1.15)
ax.set_xlim(-0.6, len(ev) - 0.4); ax.set_ylim(-1.5, 0.45); ax.axis('off')
save(fig, 'timeline')
print('ok')
