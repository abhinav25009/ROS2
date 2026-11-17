#!/usr/bin/env python3
"""Re-plan selected benchmark queries and dump the three paths as JSON,
for the path-overlay figures in the detailed report.

  source /opt/ros/humble/setup.bash
  python3 benchmark/export_example_paths.py out.json WORLD:QID [WORLD:QID ...]
"""
import contextlib
import io
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ahppa_multiworld as M                                   # noqa: E402

out, picks = sys.argv[1], [a.split(':') for a in sys.argv[2:]]
worlds = {w[0]: w for w in M.world_list('')}
feat_dep = M.ClearanceEstimator(band_z=1.0)        # 2026-11-17 version
feat_h = M.ClearanceEstimator(band_z=0.3)
M.P.set_clock(lambda: 0.0)
M.TIMEOUT_S = 30
res = []
for name in sorted({p[0] for p in picks}):
    _, fam, path, indoor, src = worlds[name]
    upright, tilted = M.load_geometry(path)
    boxes, xs, ys = M.prepare(upright, indoor)
    centers = M.shell_voxels(boxes, tilted)
    grid, octo = M.make_inputs(centers, xs, ys)
    with contextlib.redirect_stdout(io.StringIO()):
        port = M.P.PortfolioPlanner(grid, octo, safety_margin=M.SAFETY_MARGIN,
                                    unknown_penalty=M.UNKNOWN_PENALTY)
    qs = M.sample_queries(port, name, 30, boxes, xs, ys)
    for _, qid in [p for p in picks if p[0] == name]:
        sx, sy, gx, gy, d = qs[int(qid)]
        rec = dict(world=name, qid=int(qid), start=[sx, sy], goal=[gx, gy], dist=d, paths={},
                   modes={}, clearance={})

        def stock():
            return M.MultiLayerPlanner.plan(port, sx, sy, gx, gy, start_layer=M.FLY_LAYER,
                                            goal_layer=M.FLY_LAYER)

        def mk(feature):
            def run():
                M.P.FEATURE = feature
                M.P.DISPATCH = M.HysteresisDispatcher()
                M.P.WATCHDOG.reset()
                return port.plan(sx, sy, gx, gy, start_layer=M.FLY_LAYER, goal_layer=M.FLY_LAYER)
            return run
        for lab, fn in (('stock', stock), ('ahppa', mk(feat_dep)), ('ahppa_h', mk(feat_h))):
            p, ms, to = M.timed(fn)
            rec['paths'][lab] = [list(map(float, q)) for q in p] if p else None
            rec['modes'][lab] = 'A*' if lab == 'stock' else port.last_mode
            rec['clearance'][lab] = None if lab == 'stock' else float(M.P.FEATURE.last_clearance)
        res.append(rec)
        print(name, qid, {k: (len(v) if v else None) for k, v in rec['paths'].items()}, flush=True)
json.dump(res, open(out, 'w'))
