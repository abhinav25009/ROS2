#!/usr/bin/env python3
"""Replay one bump-model mission with v2, v3 and v3.1 and record every step
(positions, contacts, remembered barriers) for the demo animation.
  python3 trace_sim.py SCENARIO MISSION OUT.json"""
import argparse, json, sys, os
sys.path.insert(0, os.path.expanduser('~/Indoor-Drone-Navigation/benchmark'))
import numpy as np
import stuck_sim as SS
P, M = SS.P, SS.M
scen, mid, out = sys.argv[1], int(sys.argv[2]), sys.argv[3]
a = argparse.Namespace(n=mid + 1, only=scen, seed=0, bump=True, normal=False, bar_w=None)
name, port, centers, ghosts, qs = next(SS.ghost_worlds(a))
w = SS.GhostWorld(port, centers, ghosts, None)
pl = SS.mk_planners(w)
sx, sy, gx, gy, _ = qs[mid]
L = M.FLY_LAYER
occ = np.argwhere(w.truth[L])[::2]
walls = [[float(port.origin_x + (c + 0.5) * port.resolution), float(port.origin_y + (r + 0.5) * port.resolution)] for r, c in occ]
glass = np.concatenate([o['xy'] for o in w.objs])[::3].tolist()
res = dict(scenario=name, mission=mid, start=[sx, sy], goal=[gx, gy], walls=walls, glass=glass, runs={})
for nm in ('v2', 'v3i', 'v31'):
    tr, hits, mem, kinds = [], [], [], []
    o_obs, o_rep, o_pin, o_with = P.observe, SS.report, w.pinned, port._with_memory
    def obs(x, y, z, g, _o=o_obs):
        tr.append([round(P.CLOCK(), 2), round(x, 3), round(y, 3)])
        return _o(x, y, z, g)
    def rep(*aa, _o=o_rep, **kk):
        r = _o(*aa, **kk)
        mem.append([round(P.CLOCK(), 2), [list(map(float, s)) for s in P.memory_segments(None, 'bar', nm == 'v31')],
                    [list(map(float, s)) for s in P.memory_segments(None, 'hull', True)] if nm == 'v31' else []])
        return r
    def pin(x, y, z, _o=o_pin):
        p = _o(x, y, z)
        if p:
            hits.append([round(P.CLOCK(), 2), round(x, 3), round(y, 3)])
        return p
    def wm(kind, _o=o_with):
        kinds.append([round(P.CLOCK(), 2), kind])
        return _o(kind)
    P.observe, SS.report, w.pinned, port._with_memory = obs, rep, pin, wm
    w.rng = np.random.default_rng(1000 * mid + 7)
    real = nm in ('v3i', 'v31')
    r = SS.fly(w, pl[nm], sx, sy, gx, gy, memory=real, fast=False, real=real, bump=True)
    P.observe, SS.report, w.pinned, port._with_memory = o_obs, o_rep, o_pin, o_with
    res['runs'][nm] = dict(result=r, trace=tr, hits=hits, memory=mem, kinds=kinds)
    print(nm, r['outcome'], 'bumps', r['bumps'], 'time', r['time_s'], 'steps', len(tr), flush=True)
json.dump(res, open(out, 'w'))
print('saved', out, len(walls), 'wall pts', len(glass), 'glass pts')
