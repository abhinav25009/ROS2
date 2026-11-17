#!/usr/bin/env python3
"""Offline self-test for AHPPA Stage 1. No ROS, no simulator required.

  python3 selftest.py
"""
import random, time
import numpy as np
from density import DensityEstimator
from dispatcher import HysteresisDispatcher, OPEN, MEDIUM, DENSE, ESCAPE
from watchdog import DeadlockWatchdog
from ahppa import AHPPA

FAILS = []
def check(name, cond, detail=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {name}{'  ' + detail if detail else ''}")
    if not cond:
        FAILS.append(name)

rng = np.random.default_rng(0)
WALLS = np.array([[x, y, 1.8] for x in np.arange(-7, 7, .1) for y in (-7, 7)] +
                 [[x, y, 1.8] for y in np.arange(-7, 7, .1) for x in (-7, 7)],
                 dtype=np.float32)
def room(n):
    f = rng.uniform(-6, 6, (n, 2)).astype(np.float32)
    return np.concatenate([WALLS, np.column_stack(
        [f, np.full(len(f), 1.8, np.float32)])], 0)

print("\n-- density --")
est = DensityEstimator()
d_open, _ = est.compute(WALLS, 0, 0, 1.8)
d_full, _ = est.compute(room(30000), 0, 0, 1.8)
check("open space reads low", d_open < 0.15, f"{d_open*100:.1f}%")
check("clutter reads high", d_full > 0.50, f"{d_full*100:.1f}%")
check("clutter > open", d_full > d_open * 3)
t0 = time.perf_counter()
for _ in range(50):
    est.compute(room(30000), 0, 0, 1.8)
ms = (time.perf_counter() - t0) / 50 * 1000
check("fast enough for per-tick use", ms < 20, f"{ms:.2f} ms")
check("empty map is safe", est.compute(None, 0, 0, 1.8)[0] == 0.0)

print("\n-- hysteresis --")
def switches(lo, hi, seed=1):
    random.seed(seed)
    d = HysteresisDispatcher(min_dwell_s=0.0)
    t = 0.0
    for _ in range(200):
        d.update(random.uniform(lo, hi), now=t); t += 0.05
    return d.switches
inside = switches(0.041, 0.049)
onthresh = switches(0.046, 0.054)
random.seed(1)
naive, mode = 0, 'OPEN'
for _ in range(200):
    nxt = 'MEDIUM' if random.uniform(0.046, 0.054) > 0.05 else 'OPEN'
    if nxt != mode: naive += 1; mode = nxt
check("no switching inside the band", inside == 0, f"{inside} switches")
check("suppresses threshold chatter", onthresh <= 2,
      f"{onthresh} vs {naive} naive")
try:
    HysteresisDispatcher(open_up=0.30, open_down=0.20, dense_up=0.10, dense_down=0.05)
    bad_ok = False
except AssertionError:
    bad_ok = True
check("rejects inconsistent thresholds", bad_ok)

print("\n-- watchdog --")
w = DeadlockWatchdog(samples=30, min_displacement=0.30)
for i in range(60):
    w.update(1.0 + 0.001 * i, 2.0, 1.8, i * 0.05)
check("trips when hovering", w.tripped and w.trips == 1)
w2 = DeadlockWatchdog(samples=30, min_displacement=0.30)
for i in range(60):
    w2.update(0.05 * i, 2.0, 1.8, i * 0.05)
check("silent while flying", not w2.tripped)
w3 = DeadlockWatchdog(samples=30)
for i in range(60):
    w3.update(1.0, 2.0, 1.8, i * 0.05, goal_active=False)
check("silent with no goal", not w3.tripped)

print("\n-- orchestrator --")
def mk(ms):
    def f(sx, sy, sz, gx, gy, gz):
        time.sleep(ms / 1000.0); return [(sx, sy, sz), (gx, gy, gz)]
    return f
a = AHPPA({OPEN: mk(4), MEDIUM: mk(8), DENSE: mk(12), ESCAPE: mk(1)})
seen, t = set(), 0.0
for pts, stuck in ((WALLS, False), (room(3000), False),
                   (room(30000), False), (room(30000), True)):
    if stuck:
        a.watchdog = DeadlockWatchdog(samples=30, min_displacement=0.30)
    for i in range(35):
        m, _ = a.tick(pts, 0.0 if stuck else i * 0.4, 0.0, 1.8, t,
                      goal_active=True)
        seen.add(m); t += 0.05
    a.plan((0, 0, 1.8), (5, 5, 1.8), pts, t=t)
check("reaches every mode", seen == {OPEN, MEDIUM, DENSE, ESCAPE},
      str(sorted(seen)))
check("records latency per mode", all('ms_med' in v for k, v in a.stats().items()
                                      if not k.startswith('_')))
dwell = sum(a.dispatcher.summary().values())
check("dwell accounting sums to 100%", abs(dwell - 100.0) < 1.0, f"{dwell:.1f}%")

def boom(*_):
    raise RuntimeError("planner exploded")
b = AHPPA({OPEN: boom, MEDIUM: boom, DENSE: boom, ESCAPE: boom})
rec, path = b.plan((0, 0, 1.8), (1, 1, 1.8), WALLS, t=0.0)
check("a failing planner does not crash the stack",
      path is None and rec['error'] is not None)

print(f"\n  {'ALL PASS' if not FAILS else 'FAILURES: ' + ', '.join(FAILS)}\n")
raise SystemExit(1 if FAILS else 0)
