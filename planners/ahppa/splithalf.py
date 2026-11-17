#!/usr/bin/env python3
"""Is the density metric reproducible at a given place?

The tercile-centroid test in calib_check only catches a drone that never
moved. If high and low density interleave along the path - doorway, hall,
doorway - all three tercile centroids collapse to the middle of the
trajectory even though the metric is working perfectly.

Split-half reliability asks the question directly: bin space into cells,
split the samples of each cell randomly in two, and see whether the two
halves agree about that cell. If density is a property of place they must.

    python3 splithalf.py [csv]
"""
import csv, sys
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/density_trim.csv'
rows = list(csv.DictReader(open(path)))
f = lambda r, k: float(r[k])
d = np.array([f(r, 'density') for r in rows])
P = np.array([[f(r, 'x'), f(r, 'y'), f(r, 'z')] for r in rows])
if d.max() > 1.5:
    d = d / 100.0

CELL = 1.0
key = np.floor(P[:, :2] / CELL).astype(int)
ids = {}
lab = np.array([ids.setdefault(tuple(k), len(ids)) for k in key])
ncell = len(ids)

rng = np.random.default_rng(3)
half = rng.random(len(d)) < 0.5
A, B = [], []
for c in range(ncell):
    m = lab == c
    a, b = d[m & half], d[m & ~half]
    if len(a) >= 4 and len(b) >= 4:
        A.append(a.mean())
        B.append(b.mean())
A, B = np.array(A), np.array(B)

print()
print(f"  {len(d)} samples in {ncell} cells of {CELL} m;"
      f" {len(A)} cells had >=4 samples in both halves")

if len(A) >= 5:
    r = float(np.corrcoef(A, B)[0, 1])
    if r > 0.6:
        verdict = "density IS a property of place - calibrate from per-cell means"
    elif r < 0.3:
        verdict = "density is NOT reproducible at a given place"
    else:
        verdict = "weak - more flight time needed"
    print(f"  split-half reliability  r = {r:+.3f}")
    print(f"  -> {verdict}")
    print(f"  per-cell means span {A.min()*100:.1f}% .. {A.max()*100:.1f}%")
else:
    print("  too few well-sampled cells to judge")

grand = d.mean()
between = sum(((d[lab == c].mean() - grand) ** 2) * (lab == c).sum()
              for c in range(ncell)) / len(d)
print(f"  variance explained by location: {between / d.var() * 100:.0f}%")
print()
