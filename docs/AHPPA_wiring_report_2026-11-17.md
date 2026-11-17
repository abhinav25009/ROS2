# AHPPA Live-Wiring Fixes — Change Report

**Date:** 2026-11-17
**Scope:** `planners/ahppa/portfolio.py`, `planners/navigator_astar.py`, `planners/ahppa/density_logger.py`, `launch_sim_360.sh`, new `planners/ahppa/selftest_portfolio.py`
**Status:** All offline tests pass. **Not yet flight-tested in Gazebo.**
**Backup of the pre-change tree:** `~/IDN_backup_2026-11-17.tgz`

---

## 1. Summary

AHPPA runs in the live stack through `PortfolioPlanner`, enabled with `AHPPA_PORTFOLIO=1`. The offline orchestrator (`ahppa.py`) and its self-tests were complete, but the code path the navigator actually uses had four gaps:

| # | Problem | Effect in flight |
|---|---|---|
| 1 | `PortfolioPlanner` never passed `deadlock=` to the dispatcher, and the `DeadlockWatchdog` was not connected to the navigator | **ESCAPE mode was unreachable.** It worked only in `selftest.py` |
| 2 | The navigator never called `portfolio.set_clock()` | Minimum time in a mode was measured on the wall clock while the sim ran at ~0.63× real time (0.5 s became ~0.8 s of sim time) |
| 3 | The navigator's stuck detector, retreat timer and frontier throttle used `time.monotonic()` | Timing depended on the real-time factor, the same kind of clock-mixing bug noted three times before in `portfolio.py` |
| 4 | `density_logger.py` wrote to `/tmp` by default | The VM clears `/tmp` on reboot. The October calibration CSVs (`density_trim.csv`, `density_portfolio.csv`) were lost this way |

Two further bugs turned up while reading the code and were fixed in the same pass:

| # | Problem | Effect in flight |
|---|---|---|
| 5 | `_replan()` called the planner without `start_layer` / `goal_layer` | In `house_2floor`, a replan triggered by an obstacle could send the drone to the **wrong floor** |
| 6 | The Gazebo pose topic was hard-coded to `/world/house_2floor/...` | `launch_sim.sh` (`house_3room`) would never receive the drone's position |

---

## 2. Design

```
navigator control_loop (20 Hz)
   │  NAVIGATING: portfolio.observe(x, y, z, goal_active=True)
   │  READY:      portfolio.observe(..., goal_active=False)  → resets watchdog
   ▼
portfolio.observe()  ── down-samples to 5 Hz (sim time) ──►  DeadlockWatchdog
                                                             30 samples = 6 s window
                                                             trips if span < 0.30 m
   │
   ├─ deadlock_edge() == True (once per trip) → navigator._replan()
   ▼
PortfolioPlanner.plan()
   DISPATCH.update(tightness, deadlock=WATCHDOG.tripped, now=CLOCK())
   → OPEN: Lazy Theta*   MEDIUM: stock A*   DENSE/ESCAPE: Clearance A*
   → row appended to logs/ahppa/portfolio_<stamp>.csv
```

Design choices:

- **Watchdog state lives at module level**, next to `DISPATCH`. `MultiLayerPlanner` is rebuilt on every replan, so any state stored on the instance would be lost on each rebuild. This is the same reasoning already used for the dispatcher.
- **Down-sampling to 5 Hz.** Feeding the watchdog at 20 Hz would give a 1.5 s window, short enough to trip during an ordinary hover-turn at a waypoint. At 5 Hz the window is about 6 s.
- **One replan per deadlock.** `deadlock_edge()` returns True once per new trip. Every plan made while the watchdog stays tripped (including obstacle-triggered replans) is still dispatched in ESCAPE.
- **Watchdog only while NAVIGATING.** Frontier exploration already has its own stuck → blacklist → retreat logic, and the portfolio only acts when it is enabled. With `AHPPA_PORTFOLIO=0` the navigator behaves exactly as before, so A/B comparisons stay valid.
- **Clock fallback.** `_now()` returns node time, which is sim time with `use_sim_time:=true`. Until the first `/clock` message arrives that reads 0, so `_now()` falls back to wall time only for that startup window, which keeps a retreat timer from waiting forever.

---

## 3. Changes by file

### `planners/ahppa/portfolio.py` (~120 lines changed)
- Added module-level `WATCHDOG = DeadlockWatchdog(samples=30, min_displacement=0.30, rearm_s=5.0)` and `WATCHDOG_PERIOD_S = 0.2`.
- New `observe(x, y, z, goal_active)` feeds the watchdog on the installed clock, down-sampled, with 1 ms slack so floating-point tick sums don't stretch the window to 7.5 s.
- New `deadlock_edge()` fires once per watchdog trip.
- `_mode_for()` now passes `deadlock=WATCHDOG.tripped` (still by keyword; see the 2026-10-25 positional-argument bug note in the code).
- `plan()` records which planner was actually used, including fallbacks such as `DENSE->A*`, times the call, and writes a CSV row.
- New `_log()` writes to `logs/ahppa/portfolio_<stamp>.csv` in the repo (override with `AHPPA_LOG_DIR`). It never raises.
- `set_clock()` also sets `CLOCK_IS_SIM`, which is recorded in the log.
- The repo root is now found from the file's own location instead of `~/Desktop/Drone_IP`.

**CSV columns:** `t, sim_clock, mode, planner, deadlock, clearance, tightness, sx, sy, s_layer, gx, gy, g_layer, ms, n, ok, error`

### `planners/navigator_astar.py` (~63 lines changed)
- Keeps a handle to the portfolio module (`_ahppa`), which is `None` when the portfolio is disabled.
- New `world` parameter (default `house_2floor`) selects the Gazebo pose topic.
- New `_now()` and `_uses_sim_time()` helpers. All six `time.monotonic()` uses in the exploration, stuck, retreat and frontier logic now go through `_now()`.
- Installs `_now` as the portfolio clock at startup and prints whether it is sim or wall time.
- `control_loop`: in NAVIGATING it feeds the watchdog and calls `_replan()` once per deadlock edge; in READY it resets the watchdog.
- New `self.goal_layer` is set in `goal_cb`, passed by `_replan()`, and cleared wherever `goal_xy` is cleared.

### `planners/ahppa/density_logger.py` (~13 lines changed)
- Default `out` is now `logs/ahppa/density_<stamp>.csv` in the repo. The folder is created automatically.

### `launch_sim_360.sh` (1 line)
- The navigator now starts with `--ros-args -p use_sim_time:=true -p world:=$WORLD`.

### `planners/ahppa/selftest_portfolio.py` (new, 137 lines)
Offline test of the live wiring using a fake sim clock that does not advance with wall time.

---

## 4. Verification

| Suite | Result |
|---|---|
| `selftest.py` (density, hysteresis, watchdog, orchestrator) | ALL PASS |
| `selftest_theta.py` (Lazy Theta*) | 15 passed, 0 failed |
| `selftest_portfolio.py` (new) | **20 / 20 PASS** |

`selftest_portfolio.py` checks:
- **Watchdog:** no trip while flying at 0.5 m/s; reset when the goal is cleared; no trip before the window fills; trips after ~6 s of hovering (measured window 5.80 s); the edge fires exactly once and stays quiet while the same deadlock lasts.
- **Dispatch:** a tripped watchdog gives ESCAPE and still returns a path; the watchdog clears once the drone moves; the dispatcher then returns to OPEN in open space.
- **Clock:** minimum dwell blocks a switch 0.1 s of sim time after the last one and allows it after 1.1 s, while wall time barely moves.
- **Log:** one CSV per run, one row per plan, ESCAPE and deadlock recorded, sim clock flagged, `t` in sim seconds.

**Navigator construction check** (ROS sourced, no simulator):
- With `AHPPA_PORTFOLIO=1` and `0`, the module imports and the node constructs.
- `-p world:=house_3room` subscribes to `/world/house_3room/dynamic_pose/info`.
- `-p use_sim_time:=true` reports `clock: sim`, and the portfolio clock is the node clock.

Run the tests:

```bash
source /opt/ros/humble/setup.bash
cd ~/Indoor-Drone-Navigation/planners/ahppa
python3 selftest.py
python3 selftest_theta.py
AHPPA_PORTFOLIO=1 python3 selftest_portfolio.py
```

---

## 5. Known limitations and next steps

1. **No Gazebo flight yet.** The next flight should check whether the 6 s / 0.30 m deadlock window false-triggers during slow turns or altitude changes. Look for `deadlock=1` rows in the portfolio CSV.
2. **ESCAPE uses the same planner as DENSE** (Clearance A*). A dedicated escape planner (Informed RRT*) is planned for Stage 2.
3. **`launch_sim.sh` is unchanged.** It starts the navigators without `use_sim_time` or `world:=house_3room`. To fly `navigator_astar.py` in the 3-room house, add `--ros-args -p world:=house_3room -p use_sim_time:=true`. That launcher also starts six other navigators, which were not touched.
4. **The stuck limit is still 40 s,** but now in sim time, so it is longer in real time than before. It was raised from 8 s to compensate for wall-clock timing; re-tune it after a flight.
5. **The VM's scipy install is broken.** System scipy 1.8.0 was built for numpy 1.x, but numpy 2.2.6 is installed. `edt.py` falls back to its own chamfer EDT, so it works, but every import prints a long warning. Fix it with a numpy<2 virtualenv or a pip-installed scipy.
6. **Recalibration data needs re-collecting.** The October density CSVs were lost when `/tmp` was cleared; new runs will be saved under `logs/ahppa/`.

---

## 6. Suggested commit contents

This change sits on top of uncommitted work from Aug–Oct (AHPPA modules, two-floor world, 360° camera rig). Suggested split:

1. **`AHPPA Stage 1, two-floor world, 360 rig (WIP from Aug–Oct)`** — the previously untracked and modified files as they were before this change.
2. **`AHPPA: wire watchdog + sim clock into live portfolio, persist logs`** — the five files above plus this report.

**Backup files:** `.gitignore` ignores `*.bak`, which matches only names ending exactly in `.bak`. About 15 others (`*.bak-2floor`, `*.bak2`, `*.bak-sensors`, ...) still show as untracked. Delete them or add `*.bak*` to `.gitignore` before committing.
