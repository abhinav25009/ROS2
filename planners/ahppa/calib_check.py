#!/usr/bin/env python3
"""Is a density log spatial, or just map growth over time?

Terciles of a distribution always produce a 1/3-1/3-1/3 split, so they look
convincing even on garbage. These checks ask the question that matters:
does density change because the drone is somewhere else?

    python3 calib_check.py [path-to-csv]
"""
import csv, glob, math, os, sys
import numpy as np

path = (sys.argv[1] if len(sys.argv) > 1 else
        max(glob.glob('/tmp/density*.csv'), key=os.path.getmtime, default=''))
if not path or not os.path.exists(path):
    sys.exit("no density csv - run density_logger.py and stop it with Ctrl-C")
rows = list(csv.DictReader(open(path)))
if not rows:
    sys.exit(f"{path} has a header but no rows")
print(f"\n  file: {path}  ({len(rows)} rows)")

def col(*names):
    for n in names:
        for c in rows[0]:
            if c.strip().lower() == n:
                return c
    return None
cd, ct, cx, cy, cz = (col('density'), col('t'), col('x'), col('y'), col('z'))
if not all((cd, cx, cy, cz)):
    sys.exit(f"unexpected header: {list(rows[0])}")

def g(r, c):
    try:
        return float(r[c])
    except Exception:
        return float('nan')
d = np.array([g(r, cd) for r in rows])
P = np.array([[g(r, cx), g(r, cy), g(r, cz)] for r in rows])
t = np.array([g(r, ct) for r in rows]) if ct else np.arange(len(rows), dtype=float)
m = np.isfinite(d) & np.isfinite(P).all(1) & np.isfinite(t)
d, P, t = d[m], P[m], t[m] - t[m].min()
if d.max() > 1.5:
    d = d / 100.0
print(f"  {len(d)} usable samples, {t.max():.0f} s of sim time, "
      f"density {d.min()*100:.1f}-{d.max()*100:.1f}%\n")

gate = []
span = P.max(0) - P.min(0)
plen = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
print("  1) MOTION")
print(f"     bbox  x {span[0]:5.2f}  y {span[1]:5.2f}  z {span[2]:5.2f} m,"
      f"  path {plen:6.1f} m")
ok1 = span[0] > 4 and span[1] > 3
gate.append(ok1)
print(f"     -> {'OK, the drone roamed' if ok1 else 'TOO STATIC: not a spatial dataset'}\n")

r_t = float(np.corrcoef(d, t)[0, 1])
h1, h2 = d[t < t.max() / 2], d[t >= t.max() / 2]
print("  2) TIME CONFOUND")
print(f"     corr(density, t) = {r_t:+.3f};  first half {h1.mean()*100:.1f}%"
      f"  second half {h2.mean()*100:.1f}%")
ok2 = abs(r_t) < 0.25
gate.append(ok2)
print(f"     -> {'CLEAN' if ok2 else 'CONFOUNDED: still tracking map growth'}\n")

q = np.quantile(d, [1/3, 2/3])
print("  3) DO THE TERCILES SIT IN DIFFERENT PLACES?")
cen = []
for nm, sel in (("low ", d <= q[0]), ("mid ", (d > q[0]) & (d <= q[1])),
                ("high", d > q[1])):
    c, s = P[sel].mean(0), P[sel].std(0)
    cen.append(c)
    print(f"     {nm} n={sel.sum():4d} centroid ({c[0]:6.2f},{c[1]:6.2f},{c[2]:5.2f})"
          f" spread ({s[0]:4.2f},{s[1]:4.2f},{s[2]:4.2f})")
sep = max(math.dist(cen[i], cen[j]) for i in range(3) for j in range(i + 1, 3))
ok3 = sep > 2.0
gate.append(ok3)
print(f"     max separation {sep:.2f} m -> "
      f"{'OK' if ok3 else 'ALL ONE PLACE: same bug as the 605-sample run'}\n")

def room(p):
    x, y, z = p
    if x > 7.0:
        return 'garden'
    if z >= 3.0:
        if -1.5 < x < 0.5 and y > 2.0:
            return 'stairwell'
        return 'FF bedroom W' if x < 0 else 'FF bedroom E'
    if 1.0 < x < 5.0 and abs(y) < 2.0:
        return 'atrium column'
    if -1.5 < x < 0.5 and y > 2.0:
        return 'stairwell'
    if x > -2.0:
        return 'GF hall'
    return 'GF room S' if y < 0 else 'GF room N'

lab = [room(p) for p in P]
print("  4) DENSITY BY ROOM  (the calibration that means something)")
print(f"     {'room':<15}{'n':>5}{'p10':>8}{'median':>8}{'p90':>8}")
st = {}
for rm in sorted(set(lab)):
    sel = np.array([l == rm for l in lab])
    if sel.sum() < 15:
        continue
    v = d[sel]
    st[rm] = float(np.median(v))
    print(f"     {rm:<15}{sel.sum():5d}{np.quantile(v,.1)*100:7.1f}%"
          f"{np.median(v)*100:7.1f}%{np.quantile(v,.9)*100:7.1f}%")

if not all(gate):
    print("\n  GATE FAILED - do not set thresholds from this run.\n")
    sys.exit(0)
opn = [v for k, v in st.items() if k in ('GF hall', 'garden', 'atrium column')]
tgt = [v for k, v in st.items() if k == 'stairwell']
rms = [v for k, v in st.items() if 'bedroom' in k or k.startswith('GF room')]
print("\n  PROPOSED THRESHOLDS")
if opn and rms:
    b = (np.mean(opn) + np.mean(rms)) / 2
    print(f"     open_up   = {b+0.01:.3f}   open_down  = {b-0.01:.3f}"
          f"    (open {np.mean(opn)*100:.1f}% | rooms {np.mean(rms)*100:.1f}%)")
if rms and tgt:
    b = (np.mean(rms) + np.mean(tgt)) / 2
    print(f"     dense_up  = {b+0.015:.3f}  dense_down = {b-0.015:.3f}"
          f"   (rooms {np.mean(rms)*100:.1f}% | stairwell {np.mean(tgt)*100:.1f}%)")
else:
    print("     no stairwell samples: fly the shaft before setting dense_*,"
          " or DENSE is a regime that never fires")
print()
