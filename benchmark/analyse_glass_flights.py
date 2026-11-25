#!/usr/bin/env python3
"""Summarise glass_test PX4 flights from their logs.

A contact = tilt crosses 10 deg after >= 1 s below 5 deg while airborne
(z > 0.5 m), from pose.csv - so it is counted the same way for every
planner, whether or not the planner itself detects impacts.

  python3 benchmark/analyse_glass_flights.py LOGDIR [LOGDIR ...]
"""
import csv
import re
import sys


def summarise(L):
    lab = open(L + '/label.txt').read().split('|')[0].strip()
    P = [(float(r['t']), float(r['x']), float(r['y']), float(r['z']), float(r['tilt_deg']))
         for r in csv.DictReader(open(L + '/pose.csv'))]
    nav = re.sub(r'\x1b\[[0-9;]*m', '', open(L + '/nav.log', errors='ignore').read())
    g = re.search(r'Goal received: \(([-\d.]+), ([-\d.]+)\)', nav)
    ev, calm = [], None
    for t, x, y, z, a in P:
        if a >= 10:
            if calm is not None and t - calm >= 1.0 and z > 0.5:
                ev.append((round(t, 1), round(x, 2), round(y, 2)))
            calm = None
        elif a < 5:
            calm = t if calm is None else calm
        else:
            calm = None
    outcome = 'REACHED' if 'DESTINATION REACHED' in nav else 'CRASH' if 'CRASH:' in nav else 'timeout'
    t_goal = None
    m = re.search(r'Goal received', nav)
    # mission time: first motion after the goal to the end of motion
    moving = [t for (t, x, y, z, a), (t2, x2, y2, z2, a2) in zip(P, P[1:]) if abs(x2 - x) + abs(y2 - y) > 0.01]
    dur = (moving[-1] - moving[0]) if moving else 0.0
    obst = nav.count('Obstacle detected on path')
    return dict(label=lab, goal=f"({g.group(1)}, {g.group(2)})" if g else '?', outcome=outcome,
                contacts=len(ev), where=ev, max_tilt=max(a for *_, a in P), dur=dur,
                obstacle_replans=obst, impacts_detected=nav.count('IMPACT'),
                via_gap=any(y > 4.5 and x > 3.5 for t, x, y, z, a in P))


for L in sys.argv[1:]:
    s = summarise(L)
    print(f"{s['label']:12s} goal {s['goal']:12s} {s['outcome']:8s} contacts={s['contacts']} "
          f"(v3 detected {s['impacts_detected']})  max tilt {s['max_tilt']:5.1f}  "
          f"flight {s['dur']:5.0f} s  obstacle-replans {s['obstacle_replans']}  via gap {s['via_gap']}  {s['where']}")
