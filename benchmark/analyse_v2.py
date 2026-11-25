#!/usr/bin/env python3
"""Analyse ahppa_v2_results.csv: smoothing / corridor Lazy Theta* vs Lazy Theta*.

Times for ps and clt<R> are EXTRA time on top of stock A*; two ways to count:
    sequential   stock + extra (the drone hovers for both)
    anytime      the drone leaves on stock's path after stock's time and the
                 refinement runs while it flies: hover = stock time only
AHPPA variants are scored from the same run, using the mode the shipped
dispatcher chose for each query (ahppa_h in ahppa_multiworld_results.csv):
    shipped      OPEN -> Lazy Theta* (5 s budget, stock fallback), MEDIUM -> stock
    fix-lt       OPEN and MEDIUM -> Lazy Theta*  (point 1, the obvious fix)
    fix-ps       every mode -> smoothed A*

  python3 benchmark/analyse_v2.py V2.csv MULTIWORLD.csv [--json out.json]
"""
import csv
import json
import sys
from collections import defaultdict

import numpy as np
from scipy.stats import wilcoxon

args = [x for x in sys.argv[1:] if not x.startswith('--')]
V2, MW = args[0], args[1]
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
BUDGET_MS = 5000.0

Q = defaultdict(dict)
for r in csv.DictReader(open(V2)):
    Q[(r['world'], int(r['qid']))][r['planner']] = r
mode = {}
for r in csv.DictReader(open(MW)):
    if r['planner'] == 'ahppa_h':
        mode[(r['world'], int(r['qid']))] = r['mode']
keys = sorted(k for k, v in Q.items() if v.get('stock', {}).get('ok') == '1' and 'lt' in v and 'ps' in v)


def f(r, c):
    return float(r[c]) if r and r.get(c) not in ('', None) else np.nan


def score(k, how):
    """(length, hover_ms, min_clear) for query k under policy how."""
    v = Q[k]
    st = v['stock']
    L0, T0, C0 = f(st, 'length'), f(st, 'ms'), f(st, 'min_clear')
    if how == 'stock':
        return L0, T0, C0
    if how in ('ps', 'clt1', 'clt2', 'ps_any', 'clt1_any', 'clt2_any'):
        base = how.replace('_any', '')
        r = v.get(base)
        extra = f(r, 'ms') if r else 0.0
        if r and r['ok'] == '1':
            L, C = f(r, 'length'), f(r, 'min_clear')
        else:                                   # corridor failed -> smoothed path
            ps = v['ps']
            L, C = f(ps, 'length'), f(ps, 'min_clear')
            extra += f(ps, 'ms') if base != 'ps' else 0.0
        if base.startswith('clt'):
            extra += f(v['ps'], 'ms')           # clt needs the smoothed path first
        return L, (T0 if how.endswith('_any') else T0 + extra), C
    if how in ('lt', 'lt5'):
        r = v['lt']
        t = f(r, 'ms')
        if how == 'lt5' and t > BUDGET_MS:
            return L0, BUDGET_MS + T0, C0
        if r['ok'] == '1':
            return f(r, 'length'), t, f(r, 'min_clear')
        return L0, t + T0, C0
    m = mode.get(k, 'MEDIUM')
    if how == 'shipped':
        return score(k, 'lt5') if m == 'OPEN' else score(k, 'stock')
    if how == 'fix-lt':
        return score(k, 'lt5') if m in ('OPEN', 'MEDIUM') else score(k, 'stock')
    if how == 'fix-ps':
        return score(k, 'ps')
    raise ValueError(how)


POL = [('stock', 'Stock A*'), ('ps', 'A* + smoothing'), ('ps_any', 'A* + smoothing, anytime'),
       ('clt1', 'Corridor LT* 1 m (seq.)'), ('clt1_any', 'Corridor LT* 1 m, anytime'),
       ('clt2', 'Corridor LT* 2 m (seq.)'), ('lt', 'Lazy Theta*, no budget'), ('lt5', 'Lazy Theta*, 5 s budget'),
       ('shipped', 'AHPPA shipped'), ('fix-lt', 'AHPPA fix: MEDIUM -> LT*'), ('fix-ps', 'AHPPA fix: all -> smoothed A*')]
S = {p: np.array([score(k, p) for k in keys]) for p, _ in POL}
L0 = S['stock'][:, 0]
worlds = sorted({k[0] for k in keys})
byw = defaultdict(list)
for i, k in enumerate(keys):
    byw[k[0]].append(i)
rng = np.random.default_rng(0)


def ci(fn):
    vals = [fn(np.concatenate([byw[worlds[j]] for j in rng.integers(0, len(worlds), len(worlds))]))
            for _ in range(2000)]
    return list(np.percentile(vals, [2.5, 97.5]))


print(f"{len(keys)} queries, {len(worlds)} worlds\n")
print(f"{'policy':32s} {'length':>8s} {'median':>8s} {'p95':>8s} {'total':>8s} {'min clr':>8s} {'<0.45 m':>8s}")
out = dict(n=len(keys), table={})
for p, lab in POL:
    x = S[p]
    lp = 100 * (np.mean(x[:, 0] / L0) - 1)
    row = dict(len_pct=lp, med_ms=float(np.median(x[:, 1])), p95_ms=float(np.percentile(x[:, 1], 95)),
               total_s=float(x[:, 1].sum() / 1000), mean_clear=float(np.nanmean(x[:, 2])),
               n_close=int(np.sum(x[:, 2] < 0.45)))
    out['table'][p] = row
    print(f"{lab:32s} {lp:+7.2f}% {row['med_ms']:6.0f}ms {row['p95_ms']:6.0f}ms {row['total_s']:6.0f}s "
          f"{row['mean_clear']:7.3f}m {row['n_close']:8d}")

# how close is smoothing to Lazy Theta*?
lt_gap = S['ps'][:, 0] / S['lt'][:, 0] - 1
print(f"\nsmoothed A* vs Lazy Theta* (no budget): mean {100 * lt_gap.mean():+.2f}% "
      f"[{100 * ci(lambda i: lt_gap[i].mean())[0]:+.2f}, {100 * ci(lambda i: lt_gap[i].mean())[1]:+.2f}], "
      f"median {100 * np.median(lt_gap):+.2f}%, within 1%: {100 * np.mean(lt_gap < 0.01):.0f}%, "
      f"shorter: {100 * np.mean(lt_gap < -1e-6):.0f}%, p={wilcoxon(S['ps'][:, 0], S['lt'][:, 0]).pvalue:.1e}")
c1 = S['clt1'][:, 0] / S['lt'][:, 0] - 1
print(f"corridor LT* 1 m vs Lazy Theta*: mean {100 * c1.mean():+.3f}%, equal (<0.1%) on {100 * np.mean(np.abs(c1) < 0.001):.0f}%")
for p in ('ps', 'fix-ps', 'fix-lt', 'clt1_any'):
    t = S[p][:, 1]
    ln = S[p][:, 0] / L0 - 1
    print(f"{p:9s} vs stock: total time {100 * (t.sum() / S['stock'][:, 1].sum() - 1):+6.1f}% "
          f"{[round(100 * float(v), 1) for v in ci(lambda i: t[i].sum() / S['stock'][i, 1].sum() - 1)]}, "
          f"length {100 * ln.mean():+.2f}% {[round(100 * float(v), 2) for v in ci(lambda i: ln[i].mean())]}")
fam = defaultdict(list)
for i, k in enumerate(keys):
    fam[Q[k]['stock']['family']].append(i)
print("\nper family, length vs stock:  ps / clt1 / lt / shipped")
out['family'] = {}
for fm, idx in sorted(fam.items()):
    vals = [100 * (np.mean(S[p][idx, 0] / L0[idx]) - 1) for p in ('ps', 'clt1', 'lt', 'shipped')]
    out['family'][fm] = vals
    print(f"  {fm:6s} n={len(idx):3d}  " + "  ".join(f"{v:+6.2f}%" for v in vals))
if JSON_OUT:
    json.dump(out, open(JSON_OUT, 'w'), indent=1, default=float)
    print(f"-> {JSON_OUT}")
