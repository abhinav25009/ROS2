#!/usr/bin/env python3
"""Deadlock watchdog: 30-sample velocity history -> escape trigger.

Trips when the drone has a goal but has not meaningfully displaced across the
whole sample window. Unlike the navigator's existing stuck detector, this runs
on SIM time so it behaves identically regardless of real-time factor — the old
one used time.monotonic() and fired spuriously on a slow simulation.
"""
from collections import deque
import math


class DeadlockWatchdog:
    def __init__(self, samples=30, min_displacement=0.30, rearm_s=5.0):
        self.n = int(samples)
        self.min_disp = float(min_displacement)
        self.rearm_s = float(rearm_s)
        self.buf = deque(maxlen=self.n)
        self.tripped = False
        self.trips = 0
        self._last_trip_t = -1e9

    def reset(self):
        self.buf.clear()
        self.tripped = False

    def update(self, x, y, z, t, goal_active=True):
        """Feed one pose sample. Returns True while deadlocked."""
        if not goal_active:
            self.reset()
            return False

        self.buf.append((x, y, z, t))
        if len(self.buf) < self.n:
            return False
        if t - self._last_trip_t < self.rearm_s:
            return False

        xs = [p[0] for p in self.buf]
        ys = [p[1] for p in self.buf]
        zs = [p[2] for p in self.buf]
        span = math.sqrt((max(xs) - min(xs)) ** 2 +
                         (max(ys) - min(ys)) ** 2 +
                         (max(zs) - min(zs)) ** 2)

        if span < self.min_disp:
            if not self.tripped:
                self.trips += 1
                self._last_trip_t = t
            self.tripped = True
        else:
            self.tripped = False
        return self.tripped

    @property
    def window_s(self):
        if len(self.buf) < 2:
            return 0.0
        return self.buf[-1][3] - self.buf[0][3]
