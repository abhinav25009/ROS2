#!/usr/bin/env python3
"""Hysteresis dispatcher: pick a planning mode from obstacle density.

Two thresholds per boundary — one to go up, a lower one to come back down —
so a density hovering on a boundary cannot make the drone oscillate between
planners. Defaults follow the AHPPA proposal (5% up / 4% down); recalibrate
from measured density with set_thresholds().
"""
from collections import Counter
import time

OPEN, MEDIUM, DENSE, ESCAPE = 'OPEN', 'MEDIUM', 'DENSE', 'ESCAPE'


class HysteresisDispatcher:
    # PROVISIONAL. The 2026-09-30 calibration was INVALID: the drone never
    # moved (all samples within a 1 m box), so the three 'regimes' were
    # map growth over time, not spatial clutter. Recalibrate with
    # window_xy=6.0 and a flight that actually covers the house.
    # SUPERSEDES the note above. Recalibrated 2026-10-24 from
    # /tmp/density_trim.csv: 650 airborne in-bounds samples, 6 m window.
    # That dataset passed the checks the earlier ones failed - no time
    # confound (corr(density,t) = +0.13) and split-half reliability
    # r = +0.94 across 1 m cells, so density is reproducible at a place.
    # Breaks are weighted 1-D k-means (k=3) on per-cell means, giving
    # cluster centres 11.7 / 38.7 / 68.1 % and boundaries at 25.2 / 53.4 %.
    # The 25.2 % break independently matches where Lazy Theta* loses its
    # timing advantage in the synthetic clutter sweep (~26 %).
    # STILL PROVISIONAL: that flight covered only the hall and the atrium -
    # no bedrooms, no stairwell, no upstairs - and MEDIUM took just 9 % of
    # flight time, so the three-way split may not survive a full-house run.
    # 2026-10-25: the input is no longer density, it is TIGHTNESS derived
    # from local clearance (see clearance_feature.py). Density thresholds
    # did not transfer: across 8 third-party worlds the house-calibrated
    # values put 70 of 96 queries in OPEN and the dispatcher degenerated to
    # one planner. Clearance is physical, so these need no calibration -
    # they are just 1.0 m and 0.5 m expressed as tightness, +/-0.03.
    def __init__(self, open_up=0.363, open_down=0.303,
                 dense_up=0.697, dense_down=0.637, min_dwell_s=0.5):
        self.set_thresholds(open_up, open_down, dense_up, dense_down)
        self.min_dwell_s = float(min_dwell_s)
        self.mode = OPEN
        self.switches = 0
        # Set on the first update() so the dispatcher adopts whatever clock it
        # is driven by (sim time), rather than assuming wall time.
        self._since = None
        self._last_t = None
        self.dwell = Counter()
        self.history = []

    def set_thresholds(self, open_up, open_down, dense_up, dense_down):
        assert open_down < open_up < dense_down < dense_up, \
            "thresholds must satisfy open_down < open_up < dense_down < dense_up"
        self.open_up, self.open_down = float(open_up), float(open_down)
        self.dense_up, self.dense_down = float(dense_up), float(dense_down)

    def update(self, density, deadlock=False, now=None):
        """Return the mode to plan in. ESCAPE overrides density entirely."""
        now = time.monotonic() if now is None else now
        if self._since is None:
            # First call: classify from scratch. Without this the min-dwell
            # check (now - _since == 0) pinned the first plan of every flight
            # to the constructor default OPEN - on 2026-11-17 it planned OPEN
            # at tightness 0.90. Not counted as a switch: nothing was left.
            self._since = self._last_t = now
            self.mode = ESCAPE if deadlock else self._from_scratch(density)
            return self.mode

        if deadlock:
            self._switch(ESCAPE, now)
            return self.mode

        prev = self.mode
        if prev == ESCAPE:                       # leave escape by density
            nxt = self._from_scratch(density)
        elif prev == OPEN:
            nxt = MEDIUM if density > self.open_up else OPEN
            if density > self.dense_up:
                nxt = DENSE
        elif prev == MEDIUM:
            nxt = MEDIUM
            if density < self.open_down:
                nxt = OPEN
            elif density > self.dense_up:
                nxt = DENSE
        else:                                    # DENSE
            nxt = DENSE
            if density < self.dense_down:
                nxt = MEDIUM
            if density < self.open_down:
                nxt = OPEN

        if nxt != prev and (now - self._since) < self.min_dwell_s:
            nxt = prev                           # respect minimum dwell
        self._switch(nxt, now)
        return self.mode

    def _from_scratch(self, d):
        if d > self.dense_up:
            return DENSE
        if d > self.open_up:
            return MEDIUM
        return OPEN

    def _switch(self, nxt, now):
        # dwell accumulates every tick; _since only moves on a real change,
        # otherwise the minimum-dwell check can never be satisfied.
        self.dwell[self.mode] += max(0.0, now - self._last_t)
        self._last_t = now
        if nxt != self.mode:
            self.switches += 1
            self.history.append((round(now, 3), self.mode, nxt))
            self.mode = nxt
            self._since = now

    def summary(self):
        tot = sum(self.dwell.values()) or 1.0
        return {m: round(100 * s / tot, 1) for m, s in self.dwell.items()}
