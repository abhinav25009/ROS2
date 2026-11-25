#!/usr/bin/env python3
"""PX4 glass_test flights: per-flight summary, pooled statistics, and the
trajectory figure.

  python px4_analysis.py LOGROOT OUT_JSON
LOGROOT holds one folder per flight (pose.csv, nav.log, label.txt). The
variant comes from label.txt: AHPPA_V2 -> v2; AHPPA_V3 with
AHPPA_V3_FUSE=0 -> v3 (as flown 2026-11-19); AHPPA_V3 otherwise -> v31
(B20 flights flew before fusion existed, so they are v3).
"""
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np
from scipy.stats import fisher_exact, mannwhitneyu

ROOT, OUT = sys.argv[1], sys.argv[2]


def variant(lab):
    name, env = lab.split('|')[0].strip(), lab.split('|')[1]
    if 'AHPPA_V3' not in env:
        return 'v2'
    if name.startswith('B20-') or 'AHPPA_V3_FUSE=0' in env:
        return 'v3'
    return 'v31'


def summarise(L):
    lab = open(L + '/label.txt').read()
    P = [(float(r['t']), float(r['x']), float(r['y']), float(r['z']), float(r['tilt_deg']))
         for r in csv.DictReader(open(L + '/pose.csv'))]
    nav = re.sub(r'\x1b\[[0-9;]*m', '', open(L + '/nav.log', errors='ignore').read())
    g = re.search(r'Goal received: \(([-\d.]+), ([-\d.]+)\)', nav)
    ev, calm = [], None
    for t, x, y, z, a in P:            # contact = tilt >= 10 deg after >= 1 s below 5 deg, airborne
        if a >= 10:
            if calm is not None and t - calm >= 1.0 and z > 0.5:
                ev.append(t)
            calm = None
        elif a < 5:
            calm = t if calm is None else calm
        else:
            calm = None
    out = 'REACHED' if 'DESTINATION REACHED' in nav else 'CRASH' if 'CRASH:' in nav else 'timeout'
    name = lab.split('|')[0].strip()
    if out == 'timeout':
        if 'Replan failed' in nav:
            out = 'gave_up'          # navigator dropped the goal: no path from where it hovers
        elif name.startswith(('B20-', 'B30-')):
            out = 'cut'              # B20/B30 harness ended the wait on "No 2.5D path", which
                                     # memory relaxation prints: outcome unknown, excluded
    moving = [t for (t, x, y, *_), (t2, x2, y2, *_) in zip(P, P[1:]) if abs(x2 - x) + abs(y2 - y) > 0.01]
    return dict(label=lab.split('|')[0].strip(), variant=variant(lab), dir=os.path.basename(L),
                goal=f"({float(g.group(1)):.0f}, {float(g.group(2)):.0f})" if g else '?',
                outcome=out,
                # pose-log contacts miss a crash with no calm second before it
                # (two v2 flights): a crash in this world is always a contact
                contacts=max(len(ev), nav.count('IMPACT'), 1 if out == 'CRASH' else 0),
                contacts_pose=len(ev), impacts_detected=nav.count('IMPACT'),
                relaxed=len(re.findall(r"planned with memory '(\w+)'", nav)),
                relax_levels=sorted(set(re.findall(r"planned with memory '(\w+)'", nav))),
                dur=(moving[-1] - moving[0]) if moving else 0.0,
                via_gap=any(y > 4.5 and x > 3.5 for t, x, y, z, a in P))


ALL = [summarise(L) for L in sorted(glob.glob(ROOT + '/*')) if os.path.exists(L + '/label.txt')]
F = [f for f in ALL if f['outcome'] != 'cut']
json.dump(F, open(OUT, 'w'), indent=1)
for f in ALL:
    print(f"{f['label']:14s} {f['variant']:4s} goal {f['goal']:7s} {f['outcome']:8s} contacts {f['contacts']} "
          f"(pose {f['contacts_pose']}, detected {f['impacts_detected']})  relaxed {f['relaxed']} {f['relax_levels']}  "
          f"{f['dur']:4.0f} s  gap {f['via_gap']}")
print(f"\nexcluded (harness cut): {[f['label'] for f in ALL if f['outcome'] == 'cut']}\n")
V = ['v2', 'v3', 'v31']
tab = {}
for v in V:
    fl = [f for f in F if f['variant'] == v]
    if not fl:
        continue
    n = len(fl)
    r = sum(f['outcome'] == 'REACHED' for f in fl)
    c = sum(f['outcome'] == 'CRASH' for f in fl)
    t = n - r - c                      # gave up, or B40 time limit
    surv1 = [f for f in fl if not (f['outcome'] == 'CRASH' and f['contacts'] <= 1)]
    tab[v] = dict(n=n, reached=r, crash=c, timeout=t, contacts=float(np.mean([f['contacts'] for f in fl])),
                  survived_first=len(surv1),
                  reached_by_goal={g: f"{sum(f['outcome'] == 'REACHED' for f in fl if f['goal'] == g)}/"
                                      f"{sum(1 for f in fl if f['goal'] == g)}" for g in sorted({f['goal'] for f in fl})},
                  time_reached=float(np.mean([f['dur'] for f in fl if f['outcome'] == 'REACHED'])) if r else None)
    print(f"{v:4s} n={n:2d} reached {r:2d} timeout {t:2d} crash {c:2d}  contacts/flight {tab[v]['contacts']:.2f}  "
          f"survived first contact {len(surv1)}/{n}  by goal {tab[v]['reached_by_goal']}  "
          f"time (reached) {tab[v]['time_reached']}")
tests = {}
for a, b in (('v31', 'v3'), ('v31', 'v2'), ('v3', 'v2')):
    if a in tab and b in tab:
        A, B = tab[a], tab[b]
        p_r = fisher_exact([[A['reached'], A['n'] - A['reached']], [B['reached'], B['n'] - B['reached']]])[1]
        p_c = fisher_exact([[A['crash'], A['n'] - A['crash']], [B['crash'], B['n'] - B['crash']]])[1]
        ca = [f['contacts'] for f in F if f['variant'] == a]
        cb = [f['contacts'] for f in F if f['variant'] == b]
        tests[f'{a}_vs_{b}'] = dict(p_reached=p_r, p_crash=p_c)
        print(f"{a} vs {b}: reached {A['reached']}/{A['n']} vs {B['reached']}/{B['n']} (Fisher p={p_r:.3g}); "
              f"crashed {A['crash']}/{A['n']} vs {B['crash']}/{B['n']} (p={p_c:.3g})")
json.dump(dict(flights=F, table=tab, tests=tests), open(OUT.replace('.json', '_summary.json'), 'w'), indent=1)
