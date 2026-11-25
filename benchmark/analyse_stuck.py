#!/usr/bin/env python3
"""Analyse stuck_results.csv (benchmark/stuck_sim.py).

Per planner and scenario: success rate, median mission time (successes),
median contact time (seconds pressed against an unseen obstacle - the crash
risk), watchdog trips / fast events, memory relaxations. Paired comparison
v3f vs v2 over all missions: Wilcoxon on time (timeouts count as the limit).

  python3 benchmark/analyse_stuck.py stuck_results.csv [--json out.json]
"""
import csv
import json
import sys
from collections import defaultdict

import numpy as np
from scipy.stats import wilcoxon

src = [x for x in sys.argv[1:] if not x.startswith('--')][0]
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
rows = list(csv.DictReader(open(src)))
PL = [p for p in ('stock', 'ps', 'lt', 'ahppa', 'v2', 'v3', 'v3f') if any(r['planner'] == p for r in rows)]
NAMES = dict(stock='Stock A*', ps='A* + smoothing', lt='Lazy Theta*', ahppa='AHPPA shipped',
             v2='AHPPA v2', v3='v3 (stuck memory)', v3f='v3f (memory + fast)')
SC = list(dict.fromkeys(r['scenario'] for r in rows))
by = defaultdict(list)
for r in rows:
    by[(r['scenario'], r['planner'])].append(r)
out = dict(scenarios=SC, planners=PL, table={}, overall={}, tests={})


def stats(rs):
    ok = [r for r in rs if r['outcome'] == 'success']
    return dict(n=len(rs), success=len(ok) / len(rs),
                time=float(np.median([float(r['time_s']) for r in ok])) if ok else float('nan'),
                contact=float(np.median([float(r['contact_s']) for r in rs])),
                contact_mean=float(np.mean([float(r['contact_s']) for r in rs])),
                trips=float(np.mean([int(r['trips']) + int(r.get('fast_events') or 0) for r in rs])),
                shrinks=int(sum(int(r.get('mem_shrinks') or 0) for r in rs)),
                outcomes={o: sum(r['outcome'] == o for r in rs) for o in ('success', 'timeout', 'collision', 'no_path')})


for sc in SC:
    print(f"\n{sc}  (n={len(by[(sc, PL[0])])})")
    print(f"  {'planner':20s} {'success':>8s} {'time (ok)':>10s} {'contact med':>12s} {'trips/msn':>10s} {'relax':>6s}")
    for p in PL:
        s = stats(by[(sc, p)])
        out['table'][f'{sc}|{p}'] = s
        print(f"  {NAMES[p]:20s} {100 * s['success']:7.0f}% {s['time']:9.1f}s {s['contact']:11.1f}s "
              f"{s['trips']:10.1f} {s['shrinks']:6d}")

ghost = [s for s in SC if s != 'visible']
print("\nALL GHOST SCENARIOS")
print(f"  {'planner':20s} {'success':>8s} {'contact mean':>13s} {'contact total':>14s}  outcomes")
for p in PL:
    rs = [r for s in ghost for r in by[(s, p)]]
    st = stats(rs)
    tot = sum(float(r['contact_s']) for r in rs)
    out['overall'][p] = dict(st, contact_total=tot)
    print(f"  {NAMES[p]:20s} {100 * st['success']:7.1f}% {st['contact_mean']:12.1f}s {tot:13.0f}s  {st['outcomes']}")


def paired(a, b, scen):
    A = {(r['scenario'], r['mid']): r for s in scen for r in by[(s, a)]}
    B = {(r['scenario'], r['mid']): r for s in scen for r in by[(s, b)]}
    ks = sorted(set(A) & set(B))
    ta = np.array([float(A[k]['time_s']) for k in ks])
    tb = np.array([float(B[k]['time_s']) for k in ks])
    ca = np.array([float(A[k]['contact_s']) for k in ks])
    cb = np.array([float(B[k]['contact_s']) for k in ks])
    pt = wilcoxon(ta, tb).pvalue if np.any(ta != tb) else 1.0
    pc = wilcoxon(ca, cb).pvalue if np.any(ca != cb) else 1.0
    return dict(n=len(ks), time_a=float(ta.sum()), time_b=float(tb.sum()), p_time=float(pt),
                contact_a=float(ca.sum()), contact_b=float(cb.sum()), p_contact=float(pc),
                a_faster=float(np.mean(ta < tb)))


print("\npaired over ghost scenarios (time: timeouts count at the limit)")
for a, b in (('v3', 'v2'), ('v3f', 'v2'), ('v3f', 'v3'), ('v3f', 'stock'), ('v3f', 'lt'), ('v2', 'ahppa')):
    if a in PL and b in PL:
        t = paired(a, b, ghost)
        out['tests'][f'{a}_vs_{b}'] = t
        print(f"  {a:4s} vs {b:6s}: total time {t['time_a']:6.0f} vs {t['time_b']:6.0f} s (p={t['p_time']:.1e}), "
              f"contact {t['contact_a']:5.0f} vs {t['contact_b']:5.0f} s (p={t['p_contact']:.1e}), "
              f"{a} faster on {100 * t['a_faster']:.0f}%")
if 'visible' in SC:
    t = paired('v3f', 'v2', ['visible'])
    out['tests']['visible_v3f_vs_v2'] = t
    print(f"\ncontrol (visible railing): v3f vs v2 total time {t['time_a']:.1f} vs {t['time_b']:.1f} s; "
          f"v3f events {sum(int(r.get('fast_events') or 0) for r in by[('visible', 'v3f')])}")
if JSON_OUT:
    json.dump(out, open(JSON_OUT, 'w'), indent=1, default=float)
    print(f"-> {JSON_OUT}")
