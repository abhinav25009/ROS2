#!/usr/bin/env python3
"""Sliding-window obstacle density from the OctoMap voxel set.

AHPPA dispatches on how cluttered the space around the drone is. The metric
must be cheap enough to run every control tick, so it is computed with numpy
over the Nx3 voxel-centre array octomap_3d_query.py already maintains.

Metric: of the (x,y) cells inside a window around the drone and within the
drone's flight band in z, what fraction contain at least one occupied voxel.
This is a 2D footprint measure deliberately — it answers "how cluttered is the
space I am flying THROUGH", not "how much of this cube is solid". A full 3D
volume ratio is dominated by empty air and barely moves between an open hall
and a furnished bedroom.
"""
import numpy as np


class DensityEstimator:
    # window_xy MUST be <= the sensor range (octomap sensor_model.max_range,
    # currently 5.0 m) or most of the window is permanently unobserved and
    # the metric measures how much has been MAPPED, not how cluttered it is.
    # Measured 2026-09-30: at 15 m the reading climbed 7%% -> 23%% while the
    # drone hovered motionless, purely from OctoMap filling in.
    def __init__(self, window_xy=6.0, band_z=1.0, cell=0.25):
        """
        window_xy : side length of the square window, metres (proposal: 15)
        band_z    : half-height of the flight band checked, metres
        cell      : footprint cell size for the occupancy ratio, metres
        """
        self.window_xy = float(window_xy)
        self.band_z = float(band_z)
        self.cell = float(cell)
        self.last = None

    def compute(self, voxel_centers, x, y, z):
        """Return (density, detail). density in [0,1]; detail is a dict."""
        empty = dict(occupied_cells=0, total_cells=0, voxels_in_window=0)
        if voxel_centers is None or len(voxel_centers) == 0:
            self.last = 0.0
            return 0.0, empty

        p = np.asarray(voxel_centers, dtype=np.float32)
        h = self.window_xy * 0.5

        m = ((np.abs(p[:, 0] - x) <= h) &
             (np.abs(p[:, 1] - y) <= h) &
             (np.abs(p[:, 2] - z) <= self.band_z))
        sel = p[m]

        side = int(round(self.window_xy / self.cell))
        total = side * side
        if sel.shape[0] == 0:
            self.last = 0.0
            return 0.0, dict(occupied_cells=0, total_cells=total, voxels_in_window=0)

        cx = np.floor((sel[:, 0] - (x - h)) / self.cell).astype(np.int32)
        cy = np.floor((sel[:, 1] - (y - h)) / self.cell).astype(np.int32)
        np.clip(cx, 0, side - 1, out=cx)
        np.clip(cy, 0, side - 1, out=cy)

        occupied = np.unique(cx.astype(np.int64) * side + cy).size
        d = occupied / float(total)
        self.last = d
        return d, dict(occupied_cells=int(occupied), total_cells=total,
                       voxels_in_window=int(sel.shape[0]))
