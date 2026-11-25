---
title: "AHPPA: Fixes, Full-Mission Evaluation and Contact Memory"
subtitle: "Project report: all work to date"
date: "25 November 2026"
---

# Summary

All three points are done.

| Point | What was asked | Result |
|---|---|---|
| 1 | Fix the middle (MEDIUM) mode | **Done (AHPPA v2).** A\* followed by line-of-sight smoothing. Paths are 3.80 % shorter than stock A\*, within 0.2 % of Lazy Theta\*, and planning time is 723 s against 707 s for stock A\* over 1,320 queries. The shipped portfolio was 2.65 % shorter at 1,372 s. |
| 2 | Evaluate whole missions, not single plans | **Done.** 2,200 simulated missions on incrementally sensed maps. The deadlock watchdog never fired. Real deadlocks come from obstacles the map never holds. |
| 3 | A new algorithmic idea that beats every portfolio member | **Done (v3 contact memory + v3.1 surface fusion).** Tested in a bump-model simulation (210 missions per planner) and in 110 PX4/Gazebo flights (103 valid). |

The main results:

* **Simulation.** All map-based planners, including stock A\*, Lazy Theta\*, shipped AHPPA and v2, completed 41–42 % of missions. Each hit the unmapped obstacle about 48 times per mission. Contact memory completed 100 % with a median of one contact. Fusion cut contacts by a further 11 % (1.74 to 1.55 per mission, Wilcoxon p = 0.0013).
* **PX4 flights (103 valid).** Contact memory (v3 and v3.1, 83 flights) crashed in 61 % of flights against 95 % for v2 (p = 0.003). It reached the goal in 33 % against 5 % (p = 0.012).
* **Biggest single effect in flight.** The drone reacts to a contact by holding and backing off. With that reaction, 81 % of flights survived their first contact; v2 survived it in 25 % (p < 0.001).
* **Fusion works as designed in flight, but outcomes are not better.**
  * In flights that relaxed their memory, v3.1 hit the pane 3.0 times on average against 4.9 for v3 (Mann-Whitney p = 0.002), matching the simulation.
  * Overall outcomes did not improve: in the 60 flights flown with the fixed harness, v3.1 reached the goal 9/30 times and crashed 16/30; v3 reached it 11/30 and crashed 19/30 (p = 0.78 and 0.60).
* **New problem: giving up.** In 5 of 37 v3.1 flights, and none of 46 v3 flights, the navigator found no path from where the drone hovered and dropped the goal. All five happened while v3.1 plans exactly like v3, so fusion is not the cause; the cause is still open.
* **Test-harness bug found and fixed (21 November).** The flight script ended a flight whenever the log said "No 2.5D path found", which memory relaxation also prints. This cut 7 flights short; they are excluded.
* **Demo recordings in progress.** Gazebo and RViz demos with a live contact-memory overlay are built and tested on the VM, but nothing has been recorded yet (Section 6).

# 1. Point 1 — fixing the middle mode (v2)

The shipped MEDIUM mode switched to Lazy Theta\*. That gave shorter paths but cost almost three times the planning time. v2 keeps grid A\* and adds a line-of-sight shortcut pass over its path. The ESCAPE mode now uses clearance A\* followed by the same shortcut pass.

| Planner (1,320 queries, 44 layouts) | Path length vs stock A\* | Total planning time |
|---|---:|---:|
| Stock A\* | 0.00 % | 707 s |
| AHPPA as shipped | −2.65 % | 1,372 s |
| AHPPA, MEDIUM → Lazy Theta\* | −3.79 % | 1,886 s |
| Lazy Theta\* everywhere | −4.01 % | 2,013 s |
| **AHPPA v2 (A\* + smoothing)** | **−3.80 %** | **723 s** |

![Path length and planning time over the static benchmark.](figs/fig_static.pdf){width=100%}

# 2. Point 2 — full-mission evaluation

We ran 2,200 missions in which the map is built while the drone flies. The portfolio's watchdog never fired. Every deadlock in these missions cleared itself once the obstacle was sensed. So the escape mode only matters for obstacles the sensors never see: glass, thin posts, netting. Those became the focus of point 3.

# 3. Point 3 — contact memory (v3) and surface fusion (v3.1)

**Contact memory (v3).**

* When the drone is knocked back, tilted, or stopped by something its map says is free space, the planner writes a virtual barrier. The barrier is 4 m wide, placed across the heading, 0.3 m ahead of the contact.
* The next plan is made in ESCAPE mode.
* Barriers enter only the planner's temporary grids, never the sensed map.
* If memory seals every route, it is relaxed in steps: bar → dot → none.
* With no memory, v3 behaves exactly like v2.

**Surface fusion (v3.1).**

* In the first PX4 flights the drone hit the pane at up to 33° from its normal. Each barrier drawn across such a heading was tilted, so the next route slid past its tip into the same pane.
* v3.1 clusters contacts that lie on one line and fits that line. Contacts on an L-shaped corner are not fused.
* The cluster becomes one barrier along the fitted line.
* v3.1 adds a "hull" relaxation level: the measured span plus 0.5 m. The ladder becomes bar → hull → dot → none.
* Fusion is controlled by `AHPPA_V3_FUSE` (on by default).

![Barriers for the three contacts of PX4 flight B20-v3-10: v3 bars, v3.1 fused bar, v3.1 hull.](figs/fig_fusion.pdf){width=100%}

# 4. Simulation with a bump model

The PX4 flights showed that a contact is a bump, not a pin. The drone is thrown back 0.2–1.1 m and drops 0.2–1.8 m. In the first 20 flights, 16 of 33 contacts ended the flight.

The simulator now models this:

* A contact throws the drone back 0.5 m.
* A planner without impact detection loses 1 s.
* v3 and v3.1 instead call the navigator's own memory code, hold for 5 s, then make an ESCAPE replan.
* Each mission is scored as the probability of surviving all of its contacts, at a crash probability of 0.48 per contact.

| Planner (210 missions each) | Completed | Contacts / mission (median) | P(survive), p = 0.48 | P(survive), p = 0.2 |
|---|---:|---:|---:|---:|
| Stock A\* | 41 % | 48.43 (65) | 0.166 | 0.181 |
| Lazy Theta\* | 42 % | 47.64 (65) | 0.165 | 0.182 |
| AHPPA (shipped) | 41 % | 47.73 (66) | 0.164 | 0.182 |
| AHPPA v2 | 42 % | 47.50 (65) | 0.166 | 0.182 |
| v3 | 100 % | 1.74 (1) | 0.452 | 0.715 |
| **v3.1** | **100 %** | **1.55 (1)** | **0.460** | **0.730** |

![Bump-model simulation: contacts per mission (log scale) and survival probability.](figs/fig_bump.pdf){width=100%}

Effect of fusion (v3.1 compared with v3):

* **Glass pane scenario:** contacts fell from 3.70 to 2.60 per mission, and mission time from 50.7 to 42.5 s.
* **Per mission:** fewer contacts in 15 missions and more in 2. The worst mission fell from 8 contacts to 4; one railing mission rose from 1 to 3.
* **Relaxation:** needed in 50 missions for v3.1 against 48 for v3, but with 55 relaxation steps against 96.
* **Two-pane scenario:** no gain for either variant (3.4 contacts each). The second pane is a new surface that no memory can anticipate.

# 5. PX4/Gazebo flights

**Setup.**

* World `glass_test`: a 14 × 10 m room. At x = 4 m there is a pane the depth cameras cannot see but which is physically solid. It leaves a 2.3 m gap at one end.
* Goal is (8, 0) or (8, 2) at 1.8 m.
* Three interleaved batches:

  | Batch | Variants | Flights |
  |---|---|---:|
  | B20 | v2, v3 | 20 |
  | B30 | v2, v3, v3.1 | 30 |
  | B40 | v3, v3.1 | 20 |
  | B50 | v3, v3.1 | 40 |

**The harness bug (found 21 November).** `fly2.sh` stopped waiting, and the drone was landed, as soon as the navigator printed "No 2.5D path found". The navigator also prints this every time contact memory relaxes. So every B20/B30 flight that relaxed its memory was landed early, after 50–86 s:

* four v3 flights: B20-v3-4, B20-v3-10, B30-v3-6, B30-v3-7;
* three v3.1 flights: B30-v31-3, B30-v31-8, B30-v31-10.

I had first blamed these on a false stuck detection, but the repeated "stuck" reports came after the script had already landed the drone. The diagnosis is corrected here and in the paper. These 7 flights are excluded.

**Fixes for B40 and B50:**

* `fly2.sh` waits only for the goal or a crash;
* the limit is 360 s wall-clock (about 330 s of simulated flight);
* the PX4 battery failsafe is off (`COM_LOW_BAT_ACT 0`; it had fired in 2 flights);
* the navigator writes no stuck barrier below 0.5 m altitude.

**Results.**

| | v2 | v3 | v3.1 |
|---|---:|---:|---:|
| Valid flights | 20 | 46 | 37 |
| Reached the goal | 1 (5 %) | 14 (30 %) | 13 (35 %) |
| Crashed | 19 (95 %) | 32 (70 %) | 19 (51 %) |
| Gave up the goal | 0 | 0 | 5 |
| Survived the first contact | 5 (25 %) | 34 (74 %) | 33 (89 %) |
| Crashes per contact | 0.68 | 0.30 | 0.25 |
| Contacts per flight | 1.40 | 2.28 | 2.08 |
| Reached, goal (8, 0) / (8, 2) | 1/10, 0/10 | 9/24, 5/22 | 10/19, 3/18 |
| Mean time to goal (s) | 74 | 104 | 88 |
| **Fixed harness only (B40 + B50):** reached / crashed / gave up (of 30) | — | 11 / 19 / 0 | 9 / 16 / 5 |

"Contacts per flight" is not a quality measure: a crash ends the flight, so planners that survive contacts have more of them.

![PX4 flight outcomes and contacts per flight (all 103 valid flights).](figs/fig_px4.pdf){width=100%}

![One flight per variant: v2 crashes; v3 (B40-v3-10) relaxes to dots, hits the pane five times and crashes; v3.1 goes round the end of the pane.](figs/fig_traj.pdf){width=100%}

**Significance** (Fisher exact test unless stated):

| Comparison | Crashed | Reached the goal |
|---|---|---|
| Contact memory (v3 + v3.1) vs v2 | 51/83 vs 19/20, p = 0.003 | 27/83 vs 1/20, p = 0.012 |
| First-contact survival, memory vs v2 | 67/83 vs 5/20 survived, p < 0.001 | — |
| v3.1 vs v2 | 19/37 vs 19/20, p < 0.001 | 13/37 vs 1/20, p = 0.012 |
| v3.1 vs v3, all valid flights | 19/37 vs 32/46, p = 0.11 | 13/37 vs 14/46, p = 0.81 |
| v3.1 vs v3, fixed harness only | 16/30 vs 19/30, p = 0.60 | 9/30 vs 11/30, p = 0.78 |
| Gave up, v3.1 vs v3 | 5/37 vs 0/46, p = 0.015 | — |

**What the flights say about fusion.** Fusion only changes anything once memory relaxes. In the 60 fixed-harness flights, 14 relaxed their memory:

| | v3 (relaxes to 'dot') | v3.1 (relaxes to 'hull') |
|---|---:|---:|
| Flights that relaxed | 8 | 6 |
| Pane contacts per flight | 7, 5, 5, 5, 4, 3, 5, 5 (mean 4.9) | 3, 3, 3, 3, 3, 3 (mean 3.0) |
| Reached the goal | 5 of 8 | 2 of 6 |

So fusion does what it was built to do: v3.1 hit the pane fewer times (Mann-Whitney p = 0.002), exactly the re-contact pattern the simulation predicts. It did not turn into more goals reached (p = 0.59 for 2/6 against 5/8). With so few relaxed flights, and a crash chance of about 0.3 per contact, the expected gain is smaller than these batches can detect.

**Remaining v3.1 failures.**

* **Crashes:** most came at the first or second contact, before memory can help.
* **Gave up the goal (5 flights):** B40-v31-4, B40-v31-6, B50-v31-5, B50-v31-10, B50-v31-20. In each, the map flagged an obstacle on the path, and no plan could be found from where the drone hovered, not even by the 2D fallback, which ignores memory. So the stock navigator dropped the goal ("Replan failed — hovering").
  * Fusion cannot be the cause: one flight had no contact at all and the others had a single detected contact. A single barrier is never fused, so v3.1 planned exactly as v3 would.
  * Yet v3 never gave up in 46 flights (p = 0.015). The cause is unexplained; the next step is to look at the sensed map at the moment of failure.

**Correction to the simulator's assumption.** The 0.48 crash probability per contact is an average. Over all valid flights it is 0.68 for v2 and 0.28 for the variants that hold and back off, so the simulator understates the benefit of contact memory.

# 6. Demo recordings (in progress)

Two screen recordings of AHPPA running in Gazebo and RViz are prepared on the VM:

* **Glass pane:** the drone hits the unseen pane, remembers it and goes round.
* **House:** a four-goal tour of the two-floor house, including a climb to the first floor.

**What was built (display only; nothing feeds back into planning):**

* **RViz overlay** in `navigator_astar.py`, switched on by `AHPPA_VIZ=1` on topic `/ahppa/viz`. It shows the drone and its trail, the glass pane (translucent blue, labelled as invisible to the depth cameras), each contact point, and each remembered barrier coloured by its level ('bar', 'hull' or 'dot'), plus a status line. Tested: it publishes all layers, and the flight self-test still passes.
* **`demo_mapviz.py`**: republishes the OctoMap without the floor and ceiling so RViz can look inside the closed room.
* **`demo_flight.sh glass|house`**: brings up PX4, the Gazebo GUI, RViz and a log window, takes off, then waits for a go signal before flying the goals.
* **`demo_glass.rviz` and `demo_house.rviz`** RViz configs.
* **Gazebo view:** the camera is placed inside the room, and the pane's collision shape is drawn so viewers can see it.

**Status.**

* The glass demo was brought up on 22 November and paused before its flight. Mac screen recording works (1920×1080).
* Gazebo with its GUI runs at about 30 % of real time in UTM, so the glass flight should take about 5 minutes.
* Nothing has been recorded yet. Both demos still need: windows arranged, UTM full screen, then recording.

# 7. Limitations and next steps

1. **Finish the two demo recordings** (about 1 hour with the VM ready).
2. **Explain the give-ups.** Check the sensed map at each "Replan failed" moment. Make the navigator back off and retry instead of dropping the goal.
3. **Fusion's effect on outcomes.** The flights confirm the mechanism (fewer re-contacts) but not a better success rate. Testing that would need many more flights that relax their memory, or a world with more oblique contacts, where fusion matters more.
4. **Prevent the first contact.** This is the dominant failure and needs glass perception or a contact-tolerant airframe; contact memory cannot help with it.
5. **Hardware.** Real hardware, real glass and real depth noise are not yet tested.

# 8. Files

**Changed or new in the repository** (on the VM; nothing committed yet):

* `planners/ahppa/portfolio.py`: v2 smoothing; v3 memory (`report_stuck`, barriers, ladder); v3.1 fusion (`memory_segments`, `ladder`, `AHPPA_V3_FUSE`).
* `planners/navigator_astar.py`: impact and stuck detection; hold and back-off; calls into the memory; no stuck barrier below 0.5 m (`STUCK_MIN_Z`).
* `planners/ahppa/selftest_v2.py`, `selftest_v3.py`, `selftest_portfolio.py`, `planners/selftest_flight_safety.py`: self-tests, all passing.
* `benchmark/stuck_sim.py`: simulator with the bump model (`--bump`), and the v3i/v31 planners that call the navigator's memory code.
* `benchmark/mission_sim.py`, `flight_pose_logger.py`, analysis scripts and result CSV/JSON files.
* `worlds/glass_test.sdf`: the unseen-glass test world.
* Demo: `navigator_astar.py` overlay (`AHPPA_VIZ`), `planners/demo_mapviz.py`, `config/demo_glass.rviz`, `config/demo_house.rviz`.

**VM test scripts** (backups in `~/ahppa_backups/2026-11-21_pre-harnessfix/`): `fly2.sh` waits only for the goal or a crash; `flight_up2.sh` sets `COM_LOW_BAT_ACT 0`; `glass_batch.sh` uses a 360 s limit. Demo: `~/demo_flight.sh`, `~/demo_up.sh`.

**Paper and report** (`ahppa_paper/` on the Mac):

* `paper.md` → `paper.pdf` and `paper.tex` (first draft), via `build.sh`;
* `report.md` → this report;
* `make_figs.py`, `px4_analysis.py`, `refs.bib`, `figs/`.

Six references in `refs.bib` are marked VERIFY. They were written from memory and must be checked before submission.
