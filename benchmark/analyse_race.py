#!/usr/bin/env python3
"""Analyse ahppa_race_results.csv (measured race, 3 rotated repeats per query).

Per query, times are MEDIANS over the repeats (the VM's timing varies 2-3x
run to run). Path lengths are deterministic.

    stock        stock A* in-process
    race(w)      from the observe runs: both children timed to completion,
                 the rule applied afterwards -
                   lt done before stock           -> lt path,    time t_lt
                   lt done by (1 + w) * t_stock   -> lt path,    time t_lt
                   otherwise                      -> stock path, time (1 + w) * t_stock
                 (lt failed: stock path at max(t_stock, min(t_lt, (1+w) t_stock)))
    race0 live   the actual live code with w = 0, end to end, to check the
                 derived numbers

Confidence intervals: bootstrap over WORLDS (queries within a world are not
independent).

  python3 benchmark/analyse_race.py ahppa_race_results.csv [--json out.json]
"""
import csv
import json
import sys
from collections import defaultdict

import numpy as np
from scipy.stats import wilcoxon

src = [x for x in sys.argv[1:] if not x.startswith('--')][0]
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
WS = (0.0, 0.25, 0.5, 1.0, 2.0, float('inf'))

R = defaultdict(lambda: defaultdict(list))
meta = {}
for r in csv.DictReader(open(src)):
    k = (r['world'], int(r['qid']))
    meta[k] = (r['world'], r['family'])
    R[k][r['planner']].append(r)


def med(rows, col):
    v = [float(x[col]) for x in rows if x[col] not in ('', None)]
    return float(np.median(v)) if v else np.nan


Q = []
for k, d in R.items():
    if not all(lab in d for lab in ('stock', 'observe', 'race0')):
        continue
    st, ob, r0 = d['stock'], d['observe'], d['race0']
    q = dict(world=k[0], family=meta[k][1],
             st_ok=st[0]['ok'] == '1', st_len=med(st, 'length'), st_ms=med(st, 'ms'),
             st_clr=med(st, 'min_clear'),
             lt_ok=any(x['lt_length'] for x in ob), lt_len=med(ob, 'lt_length'),
             lt_clr=med(ob, 'lt_min_clear'),
             t_lt=1000 * med(ob, 't_lt'), t_st=1000 * med(ob, 't_stock'),
             r0_ms=med(r0, 'ms'), r0_ok=all(x['ok'] == '1' for x in r0),
             r0_len=med(r0, 'length'),
             r0_lt_share=float(np.mean([x['winner'] == 'lt' for x in r0])),
             spread=float(np.max([float(x['ms']) for x in st]) / max(1e-9, np.min([float(x['ms']) for x in st]))))
    Q.append(q)
Q = [q for q in Q if q['st_ok']]
n = len(Q)


def race(q, w):
    """(took_lt, length, ms) for wait factor w."""
    ts, tl = q['t_st'], q['t_lt']
    if q['lt_ok'] and (tl <= ts or tl <= (1 + w) * ts):
        return True, q['lt_len'], tl
    if q['lt_ok']:
        return False, q['st_len'], (1 + w) * ts
    # lt failed (budget / no path): we stop waiting when its failure arrives
    return False, q['st_len'], max(ts, min(tl if not np.isnan(tl) else ts, (1 + w) * ts))


def stats(lengths, ms, base_len):
    ratio = np.array(lengths) / np.array(base_len)
    ms = np.array(ms)
    return dict(len_pct=100 * (ratio.mean() - 1), med_ms=float(np.median(ms)),
                p95_ms=float(np.percentile(ms, 95)), total_s=float(ms.sum() / 1000))


base = [q['st_len'] for q in Q]
table = {'stock in-process': stats(base, [q['st_ms'] for q in Q], base)}
lt_alone = [(q['lt_len'] if q['lt_ok'] else q['st_len'],
             q['t_lt'] if q['lt_ok'] else q['t_lt'] + q['st_ms']) for q in Q]
table['Lazy Theta* alone (child, +fallback)'] = stats([a for a, _ in lt_alone], [b for _, b in lt_alone], base)
share = {}
for w in WS:
    rs = [race(q, w) for q in Q]
    lab = f"race w={w:g}" if w != float('inf') else 'race w=inf (always wait)'
    table[lab] = stats([x[1] for x in rs], [x[2] for x in rs], base)
    share[lab] = float(np.mean([x[0] for x in rs]))
table['race0 LIVE (measured end to end)'] = stats(
    [q['r0_len'] if q['r0_ok'] else q['st_len'] for q in Q], [q['r0_ms'] for q in Q], base)

print(f"{n} queries, {len({q['world'] for q in Q})} worlds, medians of 3 rotated repeats\n")
print(f"{'':38s} {'length':>8s} {'median':>8s} {'p95':>8s} {'total':>8s}  lt used")
for lab, s in table.items():
    print(f"{lab:38s} {s['len_pct']:+7.2f}% {s['med_ms']:6.0f}ms {s['p95_ms']:6.0f}ms {s['total_s']:6.0f}s  "
          f"{100 * share[lab]:.0f}%" if lab in share else
          f"{lab:38s} {s['len_pct']:+7.2f}% {s['med_ms']:6.0f}ms {s['p95_ms']:6.0f}ms {s['total_s']:6.0f}s")

# live vs derived agreement
d0 = np.array([race(q, 0.0)[2] for q in Q])
live = np.array([q['r0_ms'] for q in Q])
print(f"\nlive race0 vs derived w=0: median live/derived time {np.median(live / d0):.2f}; "
      f"lt-won share live {100 * np.mean([q['r0_lt_share'] for q in Q]):.0f}% vs derived "
      f"{100 * np.mean([race(q, 0)[0] for q in Q]):.0f}%")
oh = np.array([q['t_st'] - q['st_ms'] for q in Q])
print(f"stock in a child (racing) minus stock in-process: median {np.median(oh):+.0f} ms, "
      f"IQR {np.percentile(oh, 25):+.0f} to {np.percentile(oh, 75):+.0f} ms")
sp = np.array([q['spread'] for q in Q])
print(f"timing noise: max/min of the 3 in-process stock repeats, median {np.median(sp):.2f}x, 90th pct {np.percentile(sp, 90):.2f}x")

# paired tests: live race0 vs stock, and derived race(w) vs stock
print("\npaired, per query (Wilcoxon) and 95% bootstrap CI over worlds:")
byw = defaultdict(list)
for i, q in enumerate(Q):
    byw[q['world']].append(i)
worlds = list(byw)
rng = np.random.default_rng(0)
st_ms = np.array([q['st_ms'] for q in Q])
st_len = np.array(base)


def ci(fn):
    vals = []
    for _ in range(3000):
        idx = np.concatenate([byw[worlds[j]] for j in rng.integers(0, len(worlds), len(worlds))])
        vals.append(fn(idx))
    return np.percentile(vals, [2.5, 97.5])


out = dict(n=n, table=table, share=share, tests={})
for lab, ms, ln in [('race0 live', live, np.array([q['r0_len'] if q['r0_ok'] else q['st_len'] for q in Q]))] + \
        [(f'race w={w:g}', np.array([race(q, w)[2] for q in Q]), np.array([race(q, w)[1] for q in Q]))
         for w in (0.0, 0.5, 1.0)]:
    p_t = wilcoxon(ms, st_ms).pvalue
    tot = lambda idx: ms[idx].sum() / st_ms[idx].sum() - 1
    lng = lambda idx: (ln[idx] / st_len[idx]).mean() - 1
    c_t, c_l = ci(tot), ci(lng)
    print(f"  {lab:12s} total time vs stock {100 * tot(np.arange(n)):+6.1f}% [{100 * c_t[0]:+.1f}, {100 * c_t[1]:+.1f}]  "
          f"p={p_t:.1e} | length {100 * lng(np.arange(n)):+5.2f}% [{100 * c_l[0]:+.2f}, {100 * c_l[1]:+.2f}]  "
          f"| faster on {100 * np.mean(ms < st_ms):.0f}% of queries")
    out['tests'][lab] = dict(total_time_pct=100 * tot(np.arange(n)), total_time_ci=list(100 * c_t), p_time=p_t,
                             length_pct=100 * lng(np.arange(n)), length_ci=list(100 * c_l),
                             faster_share=float(np.mean(ms < st_ms)))
if JSON_OUT:
    json.dump(out, open(JSON_OUT, 'w'), indent=1, default=float)
    print(f"\n-> {JSON_OUT}")
