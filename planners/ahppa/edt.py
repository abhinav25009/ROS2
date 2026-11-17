#!/usr/bin/env python3
"""Distance transform with no scipy requirement.

scipy.ndimage would do this, but the apt scipy on the VM was built against
numpy 1.x and fails to import under numpy 2.x ("numpy.core.multiarray failed
to import"). Downgrading numpy there risks the whole ROS stack for one
function, so: use scipy when it imports, otherwise fall back to a chamfer
transform written here.

The fallback is a two-pass chamfer with Verwer's optimal 3x3 weights, about
4% worse than exact Euclidean in the worst case (0.31 cells on a 200x200
test). Immaterial for a soft cost penalty - the planner compares one cell's
clearance with its neighbour's, it does not report a measurement.

The intra-row dependency, which makes a naive chamfer a slow Python loop, is
removed with a min-plus accumulate: propagating `d[j] = min(d[j], d[j-1]+a)`
across a row equals `min over k<=j of (d[k] + a*(j-k))`, which numpy does in
one pass as `minimum.accumulate(d - a*j) + a*j`. 8.4 ms on 200x200 against
scipy's 3.5 ms.
"""
import numpy as np

A, B = 0.9619, 1.3604          # Verwer's optimal chamfer weights

try:                            # prefer scipy when the install is sane
    from scipy.ndimage import distance_transform_edt as _scipy_edt
    HAVE_SCIPY = True
except Exception:
    _scipy_edt = None
    HAVE_SCIPY = False


def _hprop(row, a=A):
    """min-plus propagation along a row, both directions."""
    n = row.shape[0]
    j = np.arange(n, dtype=np.float64)
    row = np.minimum(row, np.minimum.accumulate(row - a * j) + a * j)
    rj = j[::-1]
    row = np.minimum(row, (np.minimum.accumulate((row - a * rj)[::-1])[::-1]
                           + a * rj))
    return row


def _chamfer(free):
    """Distance in CELLS from every True cell of `free` to the nearest
    False cell, matching scipy's convention."""
    f = np.asarray(free, dtype=bool)
    H, W = f.shape
    big = float(H + W) * 2.0
    d = np.where(f, big, 0.0)

    def shift(a, k):
        o = np.full_like(a, big)
        if k > 0:
            o[k:] = a[:-k]
        elif k < 0:
            o[:k] = a[-k:]
        else:
            o[:] = a
        return o

    for i in range(H):
        if i:
            prev = d[i - 1]
            d[i] = np.minimum(d[i], np.minimum(
                prev + A, np.minimum(shift(prev, 1) + B, shift(prev, -1) + B)))
        d[i] = _hprop(d[i])
    for i in range(H - 1, -1, -1):
        if i < H - 1:
            nxt = d[i + 1]
            d[i] = np.minimum(d[i], np.minimum(
                nxt + A, np.minimum(shift(nxt, 1) + B, shift(nxt, -1) + B)))
        d[i] = _hprop(d[i])
    return d


def distance_transform_edt(free):
    if HAVE_SCIPY:
        return _scipy_edt(free)
    return _chamfer(free)
