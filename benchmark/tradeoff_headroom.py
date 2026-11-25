#!/usr/bin/env python3
"""Is there ANY objective for which per-query planner selection pays?

The length-only oracle collapsed onto "always Lazy Theta*". A drone also pays
for planning time (it hovers while planning) and wants clearance. Score every
query with

    J = length / stock_length  +  a * plan_time_s  +  b * max(0, 1 m - min_clearance)

and, for a grid of weights (a, b), compare
    best fixed   the single member with the lowest mean J over all queries
    oracle       per query, the member with the lowest J (hindsight)
    AHPPA        the shipped dispatcher
    selector     a depth-2 decision tree on features known BEFORE planning
                 (start clearance, start-goal distance, map size, obstacle
                 density), trained leave-one-FAMILY-out, so it is always
                 scored on layout types it never saw

headroom = best_fixed - oracle (in J units, i.e. "fraction of a stock path").
If headroom is ~0 for every (a, b), selection cannot help on static queries.

Timing caveat: stock / AHPPA times come from one run, Lazy Theta* / clearance
A* from another. Run-to-run noise inflates the oracle (min over noisy values),
so headroom is an UPPER bound; --noise reports the spread measured on the 884
queries where AHPPA ran Lazy Theta* in both runs.

  python3 benchmark/tradeoff_headroom.py MULTIWORLD.csv BASELINES.csv [--json out]
"""
import csv
import itertools
import json
import sys
from collections import defaultdict

import numpy as np

args = [x for x in sys.argv[1:] if not x.startswith('--')]
MW, BL = args[0], args[1]
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
MEMBERS = ('stock', 'lt', 'ca')
CLEAR_WANT = 1.0
A_GRID = (0.0, 0.003, 0.01, 0.03, 0.1, 0.3)       # per second of planning
B_GRID = (0.0, 0.1, 0.3, 1.0, 3.0)                # per metre of clearance shortfall

Q = defaultdict(dict)
for r in csv.DictReader(open(MW)):
    k = (r['world'], int(r['qid']))
    if r['planner'] == 'stock':
        Q[k].update(world=r['world'], family=r['family'], dist=float(r['dist']),
                    cells=float(r['cells']), voxels=float(r['voxels']))
        Q[k]['stock'] = r
    elif r['planner'] == 'ahppa_h':
        Q[k]['ahppa'] = r
        Q[k]['clear0'] = float(r['start_clearance'])
for r in csv.DictReader(open(BL)):
    k = (r['world'], int(r['qid']))
    Q[k][{'lt_only': 'lt', 'ca_only': 'ca'}[r['planner']]] = r
keys = sorted(k for k, v in Q.items() if all(m in v for m in MEMBERS + ('ahppa',)))


def metrics(q, m):
    """(ok, length, seconds, min_clear) with AHPPA's fallback to stock."""
    r = q[m]
    s = q['stock']
    if r['ok'] == '1':
        return float(r['length']), float(r['ms']) / 1000, float(r['min_clear'])
    if m == 'ahppa' or m == 'stock':
        return None
    return float(s['length']), (float(r['ms']) + float(s['ms'])) / 1000, float(s['min_clear'])


keys = [k for k in keys if all(metrics(Q[k], m) for m in MEMBERS + ('ahppa',))]
M = {m: np.array([metrics(Q[k], m) for k in keys]) for m in MEMBERS + ('ahppa',)}
L0 = M['stock'][:, 0]
fam = np.array([Q[k]['family'] for k in keys])
feat = np.array([[Q[k]['clear0'], Q[k]['dist'], np.log10(Q[k]['cells']),
                  Q[k]['voxels'] / Q[k]['cells']] for k in keys])
FEAT_NAMES = ('start clearance', 'distance', 'log map cells', 'voxels per cell')
MODE2M = {'OPEN': 'lt', 'MEDIUM': 'stock', 'DENSE': 'ca', 'ESCAPE': 'ca'}


def J(m, a, b):
    x = M[m]
    return x[:, 0] / L0 + a * x[:, 1] + b * np.maximum(0.0, CLEAR_WANT - x[:, 2])


# ---- depth-2 tree, exhaustive over quantile thresholds ---------------------
QS = np.linspace(0.1, 0.9, 9)


def fit_leaf(C, idx):
    return int(np.argmin(C[idx].mean(0))) if len(idx) else 0


def fit_tree(C, X, idx):
    """C: (n, members) cost matrix. Returns a callable X -> member index."""
    best = (C[idx, fit_leaf(C, idx)].sum(), ('leaf', fit_leaf(C, idx)))
    for f in range(X.shape[1]):
        for t in np.unique(np.quantile(X[idx, f], QS)):
            lo, hi = idx[X[idx, f] <= t], idx[X[idx, f] > t]
            if not len(lo) or not len(hi):
                continue
            subs = []
            for part in (lo, hi):
                sb = (C[part, fit_leaf(C, part)].sum(), ('leaf', fit_leaf(C, part)))
                for f2 in range(X.shape[1]):
                    for t2 in np.unique(np.quantile(X[part, f2], QS)):
                        a_, b_ = part[X[part, f2] <= t2], part[X[part, f2] > t2]
                        if not len(a_) or not len(b_):
                            continue
                        la, lb = fit_leaf(C, a_), fit_leaf(C, b_)
                        c = C[a_, la].sum() + C[b_, lb].sum()
                        if c < sb[0] - 1e-12:
                            sb = (c, ('split', f2, t2, ('leaf', la), ('leaf', lb)))
                subs.append(sb)
            c = subs[0][0] + subs[1][0]
            if c < best[0] - 1e-12:
                best = (c, ('split', f, t, subs[0][1], subs[1][1]))
    return best[1]


def apply_tree(node, x):
    while node[0] == 'split':
        node = node[3] if x[node[1]] <= node[2] else node[4]
    return node[1]


def tree_str(node):
    if node[0] == 'leaf':
        return MEMBERS[node[1]]
    return f"({FEAT_NAMES[node[1]]} <= {node[2]:.3g} ? {tree_str(node[3])} : {tree_str(node[4])})"


rows = []
for a, b in itertools.product(A_GRID, B_GRID):
    C = np.stack([J(m, a, b) for m in MEMBERS], 1)
    fixed = {m: float(C[:, i].mean()) for i, m in enumerate(MEMBERS)}
    best_fixed = min(fixed, key=fixed.get)
    oracle = float(C.min(1).mean())
    ah = float(J('ahppa', a, b).mean())
    rnd = float(C.mean())
    # leave-one-family-out selector
    pick = np.zeros(len(keys), int)
    for f_ in np.unique(fam):
        tr, te = np.where(fam != f_)[0], np.where(fam == f_)[0]
        tree = fit_tree(C, feat, tr)
        pick[te] = [apply_tree(tree, feat[i]) for i in te]
    sel = float(C[np.arange(len(keys)), pick].mean())
    wins = np.bincount(C.argmin(1), minlength=3)
    rows.append(dict(a=a, b=b, best_fixed=best_fixed, best_fixed_J=fixed[best_fixed],
                     oracle_J=oracle, headroom=fixed[best_fixed] - oracle,
                     ahppa_J=ah, selector_J=sel, random_J=rnd,
                     selector_gain=fixed[best_fixed] - sel, ahppa_gain=fixed[best_fixed] - ah,
                     wins=dict(zip(MEMBERS, map(int, wins))),
                     full_tree=tree_str(fit_tree(C, feat, np.arange(len(keys))))))

print(f"{len(keys)} queries, {len(set(fam))} families\n")
print("J = length/stock + a*seconds + b*max(0, 1 m - min_clear).   gains are vs the best FIXED member (+ = better)")
print(f"{'a /s':>6s} {'b /m':>5s} {'best fixed':>10s} {'J':>6s}  {'oracle headroom':>15s}  "
      f"{'AHPPA gain':>10s}  {'selector gain':>13s}  wins stock/lt/ca")
for r in rows:
    w = r['wins']
    print(f"{r['a']:6.3f} {r['b']:5.1f} {r['best_fixed']:>10s} {r['best_fixed_J']:6.3f}  "
          f"{r['headroom']:+15.4f}  {r['ahppa_gain']:+10.4f}  {r['selector_gain']:+13.4f}  "
          f"{w['stock']:4d}/{w['lt']:4d}/{w['ca']:4d}")

# ---- timing noise between the two runs --------------------------------------
op = [i for i, k in enumerate(keys) if Q[k]['ahppa']['mode'] == 'OPEN' and Q[k]['ahppa']['ok'] == '1'
      and Q[k]['lt']['ok'] == '1']
t1 = np.array([float(Q[keys[i]]['ahppa']['ms']) for i in op])
t2 = np.array([float(Q[keys[i]]['lt']['ms']) for i in op])
rat = np.log(t1 / t2)
noise = dict(n=len(op), median_ratio=float(np.exp(np.median(rat))),
             iqr_ratio=[float(np.exp(np.percentile(rat, 25))), float(np.exp(np.percentile(rat, 75)))],
             p10_p90=[float(np.exp(np.percentile(rat, 10))), float(np.exp(np.percentile(rat, 90)))],
             median_abs_diff_ms=float(np.median(np.abs(t1 - t2))))
print(f"\ntiming noise (same Lazy Theta* plan, two runs, n={noise['n']}): "
      f"AHPPA/lt time ratio median {noise['median_ratio']:.2f}, IQR {noise['iqr_ratio'][0]:.2f}-{noise['iqr_ratio'][1]:.2f}, "
      f"10-90% {noise['p10_p90'][0]:.2f}-{noise['p10_p90'][1]:.2f}; median |diff| {noise['median_abs_diff_ms']:.0f} ms")

# where is headroom largest, and what does the full-data tree look like there
top = max(rows, key=lambda r: r['headroom'])
print(f"\nlargest headroom at a={top['a']}, b={top['b']}: {top['headroom']:.4f} "
      f"(best fixed {top['best_fixed']} J={top['best_fixed_J']:.3f}); held-out selector recovers "
      f"{100 * top['selector_gain'] / top['headroom']:.0f}% of it")
print(f"  tree on all data: {top['full_tree']}")
if JSON_OUT:
    json.dump(dict(rows=rows, noise=noise, n=len(keys)), open(JSON_OUT, 'w'), indent=1)
    print(f"-> {JSON_OUT}")
