#!/usr/bin/env python3
"""Analyse mission_results.csv (closed-loop missions on maps built in flight).

Per planner: outcome shares; for missions EVERY planner completed, paired
mission time, hover (planning) time, distance flown and replans, relative to
stock A*. CIs: bootstrap over worlds.

  python3 benchmark/analyse_missions.py mission_results.csv [--json out.json]
"""
import csv
import json
import sys
from collections import Counter, defaultdict

import numpy as np
from scipy.stats import wilcoxon

src = [x for x in sys.argv[1:] if not x.startswith('--')][0]
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
R = defaultdict(dict)
for r in csv.DictReader(open(src)):
    R[(r['world'], int(r['mid']))][r['planner']] = r
PL = [p for p in ('stock', 'ps', 'lt', 'ahppa', 'v2') if any(p in v for v in R.values())]
NAMES = dict(stock='Stock A*', ps='A* + smoothing', lt='Lazy Theta*', ahppa='AHPPA shipped', v2='AHPPA v2')
keys = sorted(k for k, v in R.items() if all(p in v for p in PL))
print(f"{len(keys)} missions, {len({k[0] for k in keys})} worlds, planners {PL}\n")

out = dict(n=len(keys), outcomes={}, paired={}, tests={})
print(f"{'planner':16s} {'success':>8s} {'collision':>9s} {'timeout':>8s} {'no path':>8s}  deadlock trips  collisions with obstacle already mapped")
for p in PL:
    c = Counter(R[k][p]['outcome'] for k in keys)
    n = len(keys)
    trips = sum(int(R[k][p]['trips']) for k in keys)
    known = sum(1 for k in keys if R[k][p]['outcome'] == 'collision' and 'seen_near=1' in R[k][p].get('collision', ''))
    out['outcomes'][p] = {o: c[o] / n for o in ('success', 'collision', 'timeout', 'no_path')}
    out['outcomes'][p]['trips'] = trips
    print(f"{NAMES[p]:16s} {100 * c['success'] / n:7.1f}% {100 * c['collision'] / n:8.1f}% "
          f"{100 * c['timeout'] / n:7.1f}% {100 * c['no_path'] / n:7.1f}%  {trips:14d}  {known}")

both = [k for k in keys if all(R[k][p]['outcome'] == 'success' for p in PL)]
print(f"\nmissions every planner completed: {len(both)}")
f = lambda p, c: np.array([float(R[k][p][c]) for k in both])
T0, H0, F0 = f('stock', 'time_s'), f('stock', 'plan_s'), f('stock', 'flown')
worlds = sorted({k[0] for k in both})
byw = defaultdict(list)
for i, k in enumerate(both):
    byw[k[0]].append(i)
rng = np.random.default_rng(0)


def ci(fn):
    v = [fn(np.concatenate([byw[worlds[j]] for j in rng.integers(0, len(worlds), len(worlds))]))
         for _ in range(2000)]
    return [float(x) for x in np.percentile(v, [2.5, 97.5])]


print(f"{'planner':16s} {'mission time':>13s} {'hover (plan)':>13s} {'flown':>8s} {'replans':>8s}   time vs stock [95% CI]")
for p in PL:
    T, Hh, F, Rp = f(p, 'time_s'), f(p, 'plan_s'), f(p, 'flown'), f(p, 'replans')
    rel = lambda i: T[i].sum() / T0[i].sum() - 1
    c = ci(rel)
    out['paired'][p] = dict(time_s=float(np.median(T)), plan_s=float(np.median(Hh)), flown=float(np.median(F)),
                            replans=float(np.median(Rp)), total_time=float(T.sum()), total_plan=float(Hh.sum()),
                            total_flown=float(F.sum()), time_vs_stock=float(rel(np.arange(len(both)))), ci=c)
    print(f"{NAMES[p]:16s} {np.median(T):11.1f} s {np.median(Hh):11.2f} s {np.median(F):6.1f} m {np.median(Rp):8.0f}"
          f"   {100 * rel(np.arange(len(both))):+6.1f}% [{100 * c[0]:+.1f}, {100 * c[1]:+.1f}]"
          f"   totals: {T.sum():.0f} s, hover {Hh.sum():.0f} s, flown {F.sum():.0f} m")
for p in PL:
    if p == 'stock':
        continue
    T = f(p, 'time_s')
    pv = wilcoxon(T, T0).pvalue if np.any(T != T0) else 1.0
    out['tests'][p] = dict(p=pv, faster_share=float(np.mean(T < T0)))
    print(f"  {NAMES[p]:16s} vs stock: Wilcoxon p={pv:.1e}, faster on {100 * np.mean(T < T0):.0f}% of missions")
if 'v2' in PL and 'ahppa' in PL:
    T1, T2 = f('ahppa', 'time_s'), f('v2', 'time_s')
    print(f"  AHPPA v2 vs shipped: total {100 * (T2.sum() / T1.sum() - 1):+.1f}%, p={wilcoxon(T2, T1).pvalue:.1e}")
fam = defaultdict(list)
for k in keys:
    fam[R[k]['stock']['family']].append(k)
print("\nsuccess by family:  " + "  ".join(f"{p:>6s}" for p in PL))
out['family'] = {}
for fm, ks in sorted(fam.items()):
    s = [np.mean([R[k][p]['outcome'] == 'success' for k in ks]) for p in PL]
    out['family'][fm] = s
    print(f"  {fm:6s} n={len(ks):3d} " + "  ".join(f"{100 * v:5.0f}%" for v in s))
if JSON_OUT:
    json.dump(out, open(JSON_OUT, 'w'), indent=1, default=float)
    print(f"-> {JSON_OUT}")
