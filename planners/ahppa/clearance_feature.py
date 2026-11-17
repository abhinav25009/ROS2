#!/usr/bin/env python3
"""Local clearance as the dispatch feature, replacing density.

WHY THE CHANGE
    Density - the fraction of nearby footprint cells that are occupied -
    does not transfer between environments. Measured across 8 third-party
    worlds, the thresholds calibrated on house_2floor put 70 of 96 queries
    in OPEN, so the dispatcher degenerated to a single planner.

    Density answers "how much obstacle is around me?" while what predicts a
    collision is "how tight is the gap I must fly through?".
    SC3_dead_end_comb has density 0.59 at 0.65 m clearance; SC7b_clutter
    has density 0.25 at 0.60 m clearance. Same tightness, different density.

    Clearance is in metres, so a threshold means something physical relative
    to the drone's radius and needs no per-world calibration. Dispatching on
    it gave a balanced 36/25/35 mode split across those 8 worlds, untuned.

WHAT THE DISPATCHER SEES
    HysteresisDispatcher expects a scalar in [0, 1] where HIGHER is TIGHTER.
    Rather than invert its state machine, convert:

        tightness = clamp((c_open - clearance) / c_open, 0, 1)

    With c_open = 1.5 m:  1.5 m -> 0.00   1.0 m -> 0.33
                          0.5 m -> 0.67   0.0 m -> 1.00

VERTICAL BAND: +/-0.3 m, NOT +/-1.0 m (changed 2026-11-17)
    Only voxels within band_z of the drone's altitude count. The original
    +/-1.0 m band reached 2.8 m from the 1.8 m flight layer - exactly where a
    normal ceiling is - so indoors the nearest "obstacle" was the ceiling and
    the reading sat at 0.95 m, 5 mm under the OPEN threshold. In the 44-layout
    benchmark (docs/AHPPA_detailed_report.pdf) that pinned AHPPA to MEDIUM on
    1,109 of 1,110 indoor queries; raising the ceiling 10 cm flipped 76 of
    120 to OPEN. Under a SLOPING ceiling (house_2floor's roof is pitched
    12.6 deg) the reading would change continuously along the slope, so the
    mode would depend on where under the roof the drone is, not on the walls.

    +/-0.3 m is the planner's own collision band (DRONE_CLEARANCE in
    planner_3d.py): an obstacle outside it cannot block the current layer,
    so it should not decide the mode either. A ceiling or roof slope that
    comes within 0.3 m of the flight altitude still counts - there it is a
    genuine obstacle at flight height. Benchmark with the 0.3 m band:
    2.8% shorter paths, 59% fewer waypoints, no clearance loss.

LIMITATION, STATED PLAINLY
    Clearance comes from OctoMap's occupied voxels, which exist only where
    the drone has already seen something, so unobserved space reads as wide
    open. Same optimistic-unknown assumption the planner makes. Acceptable
    for choosing a mode; it under-reports tightness on a first pass.
"""
import numpy as np

C_OPEN_DEFAULT = 1.5          # metres; clearance at or above this is "open"
BAND_Z_DEFAULT = 0.3          # metres; = planner_3d.DRONE_CLEARANCE (see above)


class ClearanceEstimator:
    """Distance from the drone to the nearest occupied voxel in its band."""

    def __init__(self, band_z=BAND_Z_DEFAULT, max_range=5.0, c_open=C_OPEN_DEFAULT):
        self.band_z = float(band_z)
        self.max_range = float(max_range)
        self.c_open = float(c_open)
        self.last_clearance = float('inf')
        self.last_n = 0

    def clearance(self, voxel_centers, pos):
        """Metres to the nearest occupied voxel within the altitude band,
        capped at max_range. No scipy needed: filtering to a box around the
        drone first keeps this as cheap as the old density call."""
        x, y, z = pos
        if voxel_centers is None or len(voxel_centers) == 0:
            self.last_clearance, self.last_n = self.max_range, 0
            return self.max_range
        p = np.asarray(voxel_centers, dtype=np.float32)
        r = self.max_range
        m = ((np.abs(p[:, 0] - x) <= r) & (np.abs(p[:, 1] - y) <= r) &
             (np.abs(p[:, 2] - z) <= self.band_z))
        sel = p[m]
        self.last_n = int(sel.shape[0])
        if sel.shape[0] == 0:
            self.last_clearance = self.max_range
            return self.max_range
        d = np.sqrt((sel[:, 0] - x) ** 2 + (sel[:, 1] - y) ** 2
                    + (sel[:, 2] - z) ** 2)
        c = float(min(d.min(), self.max_range))
        self.last_clearance = c
        return c

    def tightness(self, voxel_centers, pos):
        """The [0,1] value the dispatcher consumes. Higher = tighter."""
        c = self.clearance(voxel_centers, pos)
        return float(np.clip((self.c_open - c) / self.c_open, 0.0, 1.0))

    @staticmethod
    def tightness_of(clear_m, c_open=C_OPEN_DEFAULT):
        return float(np.clip((c_open - clear_m) / c_open, 0.0, 1.0))


# The dispatcher thresholds that follow from 1.0 m and 0.5 m clearance.
OPEN_UP = round(ClearanceEstimator.tightness_of(1.00) + 0.03, 3)
OPEN_DOWN = round(ClearanceEstimator.tightness_of(1.00) - 0.03, 3)
DENSE_UP = round(ClearanceEstimator.tightness_of(0.50) + 0.03, 3)
DENSE_DOWN = round(ClearanceEstimator.tightness_of(0.50) - 0.03, 3)

if __name__ == '__main__':
    import time
    e = ClearanceEstimator()
    print(f"\n  c_open = {e.c_open} m\n")
    print(f"  {'clearance':>10}  {'tightness':>10}  mode")
    for c in (3.0, 2.0, 1.5, 1.2, 1.0, 0.8, 0.6, 0.5, 0.4, 0.2, 0.0):
        t = ClearanceEstimator.tightness_of(c)
        mode = 'OPEN' if t < OPEN_UP else ('MEDIUM' if t < DENSE_UP else 'DENSE')
        print(f"  {c:9.2f} m  {t:10.3f}  {mode}")
    print(f"\n  open_up   = {OPEN_UP}   open_down  = {OPEN_DOWN}")
    print(f"  dense_up  = {DENSE_UP}   dense_down = {DENSE_DOWN}")
    assert OPEN_DOWN < OPEN_UP < DENSE_DOWN < DENSE_UP, "ordering broken"
    print("  ordering OK")
    rng = np.random.default_rng(0)
    vox = rng.uniform(-10, 10, size=(120000, 3)).astype(np.float32)
    t0 = time.perf_counter()
    for _ in range(50):
        e.tightness(vox, (0.0, 0.0, 1.5))
    print(f"  {((time.perf_counter()-t0)/50)*1000:.2f} ms per call\n")
