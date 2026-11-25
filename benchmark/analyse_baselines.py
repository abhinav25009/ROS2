#!/usr/bin/env python3
"""Oracle / random / fixed-planner comparison for AHPPA.

Joins ahppa_multiworld_results.csv (stock, ahppa_h) with
ahppa_baselines_results.csv (lt_only, ca_only) on (world, qid).

Selectors compared, all with AHPPA's own fallback (member fails -> stock A*,
time = member time + stock time):
    stock      always stock A*                 (what AHPPA calls MEDIUM)
    lt         always Lazy Theta*              (OPEN)
    ca         always clearance A*             (DENSE / ESCAPE)
    random     uniform over the three members  (expected value, exact)
    AHPPA      the shipped dispatcher (ahppa_h, +/-0.3 m band)
    oracle     per query, the member with the shortest path, in hindsight

Headline: the share of the oracle's length saving over stock that AHPPA
captures, with a 95% bootstrap CI resampling WORLDS (queries inside a world
are not independent).

  python3 benchmark/analyse_baselines.py [multiworld.csv] [baselines.csv] [--json out.json]
"""
import csv
import json
import os
import sys
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
args = [x for x in sys.argv[1:] if not x.startswith('--')]
MW = args[0] if len(args) > 0 else os.path.join(HERE, 'ahppa_multiworld_results.csv')
BL = args[1] if len(args) > 1 else os.path.join(HERE, 'ahppa_baselines_results.csv')
JSON_OUT = sys.argv[sys.argv.index('--json') + 1] if '--json' in sys.argv else None
MEMBERS = ('stock', 'lt', 'ca')
FAM_NAME = {'F1': 'mazes', 'F2': 'offices', 'F3': 'aisles', 'F4': 'forests',
            'F5': 'urban', 'MO1': 'office (MO)', 'MO2': 'warehouse (MO)', 'repo': 'repo houses'}


def num(v):
    return float(v) if v not in ('', None) else None


Q = defaultdict(dict)
for r in csv.DictReader(open(MW)):
    if r['planner'] in ('stock', 'ahppa_h'):
        k = (r['world'], int(r['qid']))
        Q[k]['family'] = r['family']
        Q[k]['world'] = r['world']
        Q[k]['indoor'] = r['indoor']
        lab = 'stock' if r['planner'] == 'stock' else 'ahppa'
        Q[k][lab] = dict(ok=r['ok'] == '1', len=num(r['length']), ms=float(r['ms']),
                         clr=num(r['min_clear']), mode=r['mode'])
for r in csv.DictReader(open(BL)):
    k = (r['world'], int(r['qid']))
    if k not in Q:
        continue
    lab = {'lt_only': 'lt', 'ca_only': 'ca'}[r['planner']]
    Q[k][lab] = dict(ok=r['ok'] == '1', len=num(r['length']), ms=float(r['ms']),
                     clr=num(r['min_clear']), to=r['timeout'] == '1')
Q = {k: v for k, v in Q.items() if all(m in v for m in MEMBERS + ('ahppa',))}
worlds = sorted({k[0] for k in Q})


def fb(q, m):
    """member m with AHPPA's fallback to stock."""
    r = q[m]
    if r['ok'] or m == 'stock':
        return dict(r)
    s = q['stock']
    return dict(ok=s['ok'], len=s['len'], ms=r['ms'] + s['ms'], clr=s['clr'])


def selectors(q):
    out = {m: fb(q, m) for m in MEMBERS}
    out['AHPPA'] = q['ahppa']
    ok = [out[m] for m in MEMBERS if out[m]['ok']]
    out['oracle'] = min(ok, key=lambda r: r['len']) if ok else dict(ok=False, len=None, ms=0, clr=None)
    # uniform random member: expectation over the three choices
    out['random'] = dict(ok=np.mean([out[m]['ok'] for m in MEMBERS]) == 1.0,
                         p_ok=float(np.mean([out[m]['ok'] for m in MEMBERS])),
                         len=float(np.mean([out[m]['len'] for m in MEMBERS])) if all(out[m]['ok'] for m in MEMBERS) else None,
                         ms=float(np.mean([out[m]['ms'] for m in MEMBERS])),
                         clr=float(np.mean([out[m]['clr'] for m in MEMBERS])) if all(out[m]['ok'] for m in MEMBERS) else None)
    return out


SEL = ('stock', 'lt', 'ca', 'random', 'AHPPA', 'oracle')
S = {k: selectors(q) for k, q in Q.items()}
# length comparisons only where every selector found a path
common = [k for k in S if all(S[k][s]['ok'] for s in SEL)]


def summary(keys):
    res = {}
    for s in SEL:
        rows = [S[k][s] for k in keys]
        ok = [r.get('p_ok', float(r['ok'])) for r in rows]
        c = [S[k][s] for k in keys if k in common_set]
        st = [S[k]['stock'] for k in keys if k in common_set]
        ratio = [a['len'] / b['len'] for a, b in zip(c, st)]
        res[s] = dict(success=float(np.mean(ok)) if rows else None,
                      len_vs_stock_pct=100 * (float(np.mean(ratio)) - 1) if ratio else None,
                      total_len=float(sum(r['len'] for r in c)),
                      median_ms=float(np.median([r['ms'] for r in rows])) if rows else None,
                      p95_ms=float(np.percentile([r['ms'] for r in rows], 95)) if rows else None,
                      mean_clear=float(np.mean([r['clr'] for r in c])) if c else None,
                      n_common=len(c))
    return res


common_set = set(common)


def captured(keys):
    """share of the oracle's total length saving over stock that AHPPA gets."""
    ks = [k for k in keys if k in common_set]
    st = sum(S[k]['stock']['len'] for k in ks)
    ah = sum(S[k]['AHPPA']['len'] for k in ks)
    orc = sum(S[k]['oracle']['len'] for k in ks)
    return (st - ah) / (st - orc) if st - orc > 1e-9 else float('nan'), st, ah, orc


def boot_captured(n=4000, seed=0):
    rng = np.random.default_rng(seed)
    byw = defaultdict(list)
    for k in common:
        byw[k[0]].append(k)
    ws = list(byw)
    vals = []
    for _ in range(n):
        pick = rng.choice(len(ws), len(ws))
        st = ah = orc = 0.0
        for i in pick:
            for k in byw[ws[i]]:
                st += S[k]['stock']['len']
                ah += S[k]['AHPPA']['len']
                orc += S[k]['oracle']['len']
        if st - orc > 1e-9:
            vals.append((st - ah) / (st - orc))
    return np.percentile(vals, [2.5, 97.5])


# which member was best, and how often AHPPA chose it
MODE2M = {'OPEN': 'lt', 'MEDIUM': 'stock', 'DENSE': 'ca', 'ESCAPE': 'ca'}
pick_stats = defaultdict(lambda: defaultdict(int))
for k in common:
    q = Q[k]
    best = min(MEMBERS, key=lambda m: S[k][m]['len'])
    # ties within 1%: any member within 1% of the best counts as right
    good = {m for m in MEMBERS if S[k][m]['len'] <= 1.01 * S[k][best]['len']}
    chose = MODE2M.get(q['ahppa']['mode'].split('-')[0], 'stock')
    pick_stats['best'][best] += 1
    pick_stats['chose'][chose] += 1
    pick_stats['right'][chose in good] += 1
    pick_stats['tie_all'][len(good) == 3] += 1

# consistency: AHPPA in OPEN should reproduce lt_only exactly
mism = [k for k in Q if Q[k]['ahppa']['mode'] == 'OPEN' and Q[k]['ahppa']['ok'] and Q[k]['lt']['ok']
        and abs(Q[k]['ahppa']['len'] - Q[k]['lt']['len']) > 0.01]

out = dict(n_queries=len(Q), n_worlds=len(worlds), n_common=len(common),
           overall=summary(list(Q)), captured=captured(list(Q))[0],
           captured_ci=list(boot_captured()), pick={a: dict(b) for a, b in pick_stats.items()},
           open_mismatch=len(mism), timeouts={m: sum(Q[k][m].get('to', False) for k in Q) for m in ('lt', 'ca')},
           raw_fail={m: sum(not Q[k][m]['ok'] for k in Q) for m in MEMBERS},
           families={})
fams = sorted({Q[k]['family'] for k in Q})
for fm in fams:
    ks = [k for k in Q if Q[k]['family'] == fm]
    out['families'][fm] = dict(name=FAM_NAME.get(fm, fm), n=len(ks), worlds=len({k[0] for k in ks}),
                               sel=summary(ks), captured=captured(ks)[0])
# per-world oracle saving (to see where selection could matter at all)
out['worlds'] = {}
for wname in worlds:
    ks = [k for k in common if k[0] == wname]
    if ks:
        st = sum(S[k]['stock']['len'] for k in ks)
        out['worlds'][wname] = dict(
            family=Q[ks[0]]['family'], n=len(ks),
            oracle_pct=100 * (sum(S[k]['oracle']['len'] for k in ks) / st - 1),
            ahppa_pct=100 * (sum(S[k]['AHPPA']['len'] for k in ks) / st - 1),
            lt_pct=100 * (sum(S[k]['lt']['len'] for k in ks) / st - 1),
            ca_pct=100 * (sum(S[k]['ca']['len'] for k in ks) / st - 1))

o = out['overall']
print(f"\n{out['n_queries']} queries, {out['n_worlds']} worlds; {out['n_common']} where every selector succeeds")
print(f"raw member failures: {out['raw_fail']}   timeouts: {out['timeouts']}   OPEN/lt mismatches: {len(mism)}\n")
print(f"{'selector':8s} {'success':>8s} {'len vs stock':>13s} {'median ms':>10s} {'p95 ms':>9s} {'min clear':>10s}")
for s in SEL:
    r = o[s]
    print(f"{s:8s} {100 * r['success']:7.1f}% {r['len_vs_stock_pct']:+12.2f}% {r['median_ms']:10.0f} "
          f"{r['p95_ms']:9.0f} {r['mean_clear']:9.3f}m")
lo, hi = out['captured_ci']
print(f"\nAHPPA captures {100 * out['captured']:.1f}% of the oracle's length saving "
      f"(95% CI {100 * lo:.1f} to {100 * hi:.1f}%, bootstrap over worlds)")
print(f"best member per query: {dict(pick_stats['best'])}")
print(f"AHPPA chose:          {dict(pick_stats['chose'])}")
print(f"AHPPA chose a best-within-1% member: {pick_stats['right'][True]}/{len(common)};  "
      f"all three within 1%: {pick_stats['tie_all'][True]}")
print(f"\n{'family':16s} {'n':>4s} {'oracle':>8s} {'AHPPA':>8s} {'lt':>8s} {'ca':>8s} {'random':>8s} {'captured':>9s}")
for fm, d in out['families'].items():
    s_ = d['sel']
    f_ = lambda x: f"{x:+7.2f}%" if x is not None else '     n/a'
    print(f"{d['name']:16s} {d['n']:4d} {f_(s_['oracle']['len_vs_stock_pct'])} {f_(s_['AHPPA']['len_vs_stock_pct'])} "
          f"{f_(s_['lt']['len_vs_stock_pct'])} {f_(s_['ca']['len_vs_stock_pct'])} {f_(s_['random']['len_vs_stock_pct'])} "
          f"{100 * d['captured']:8.1f}%")
if JSON_OUT:
    json.dump(out, open(JSON_OUT, 'w'), indent=1, default=float)
    print(f"\n-> {JSON_OUT}")
