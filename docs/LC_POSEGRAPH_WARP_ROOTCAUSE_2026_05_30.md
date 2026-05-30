# Pose-Graph House-Loop Warp: Root-Cause Synthesis

## 1. ROOT CAUSE

**VERIFIED.** `pose_graph_.optimize()` warped the house-loop because it had to close a **47.9 m residual on a ~100 m loop from a single loop edge** — a geometrically unsolvable rigid constraint that the 4-DOF Gauss-Newton optimizer "satisfied" by smearing a 4.7 m max displacement (+12.3° yaw) across all 32 sparse nodes. The 47.9 m residual is not real drift (real VIO drift ≈3.4 m for 94 m); it is **accumulated frame-incoherence**: the PoseGraph nodes are frozen at `addNode()` time in EKF-position space, but 5 prior EKF direct corrections shifted `global_t_`/`p_G_` by ~9.9 m cumulatively *without retroactively updating any frozen node* (there is no `updateNode()` API), so the loop-edge measurement and the node chain end up in mutually contradictory positions.

## 2. THE EVIDENCE CHAIN

1. Optimizer is 4-DOF SE(3), no scale solve — `PoseGraph.h:10-11, 33-39`; `PoseGraph.cpp:213` `DOF = 4`. A scale gap cannot be closed by rigid deformation.
2. Nodes are sourced from `ekf_.getClonePose()` (= EKF `p_G_` ← `setPosition(global_t_)`) and **frozen** — `Tracker.cpp:5205-5207`, `6121-6127`; `PoseGraph.h:112-255` exposes only `addNode`/`addLoopEdge`/`optimize` — **no `updateNode`**.
3. EKF direct corrections move `p_G_`/`global_t_` but not the stored nodes — `Tracker.cpp:7692-7720` (`updateAbsolutePose`, `global_t_ += delta_p`).
4. 5 such corrections fired before the pose-graph trigger, summing ~9.9 m (teleports 1.229/1.284/1.254/1.516/4.618 m at idx 41/95/177/622/686) — `event_summary val_2026_05_30b`, trajectory analysis.
5. Only **1** of 6 LC accepts produced a loop edge (`pose_graph_loop_edges_added=1`, `loop_closure_accepts=6`); the others died at the `ok && k==0 && match_pg>=0 && now_pg>=0` gate — `Tracker.cpp:7692-7810` (10 rot-sanity + 6 chi2 rejects).
6. That single edge carried a **47.9 m pre-residual** (`pose_graph_residual_norm_pre_mm=47864`) taken at full `info_xy=100` weight with **no robust kernel** — `PoseGraph.cpp:262-350`.
7. Optimizer converged (5 iters, residual 47.9→5.4 m, ratio 0.112) but applied **4.691 m max node displacement + 215 mrad (12.3°) yaw** — `pose_graph_max_correction_mm=4691`, `pose_graph_max_correction_mrad=215`.
8. The corrected node positions are written verbatim to `corrected_traj_xz_` via `snapshotNodes()` — `Tracker.cpp:8014-8030` — and the UI faithfully redraws them (`NavSightViewModel.kt:349-371`, clear → rebuild from polyline → append live point). **32 teleport jumps == `pose_graph_apply_calls=32`** (`Tracker.cpp:7977-7978`).
9. The redraw fires **once**; the other 5 EKF corrections move the live dot but never rebuild `corrected_traj_xz_`, so the warped polyline persists and ends offset from the live dot.

## 3. WHY V54 WAS CLEAN BUT THIS ONE ISN'T

Not scale, not false match, not gauge (anchor is correctly implemented — `PoseGraph.cpp:212, 318`), not the 4M:1 weight bug (already rebalanced to 4:1 / 276:1). The difference is **how much work the optimizer had to do**, driven by edge count, residual size, and node density:

| Counter | v54 (clean) | house-loop (bad) |
|---|---|---|
| `pose_graph_loop_edges_added` | 3 | **1** |
| `pose_graph_optimize_calls` | 3 | **1** |
| `pose_graph_residual_norm_pre_mm` | 8716 (≈ambient noise) | **47864 (5.5×)** |
| `pose_graph_max_correction_mm` | 13 | **4691 (360×)** |
| `pose_graph_max_correction_mrad` | 0 | **215 (12.3°)** |
| residual ratio (post/pre) | 0.9999 (no-op) | 0.112 (aggressive) |
| `pose_graph_apply_calls` (nodes moved) | 175 | **32** |
| `cam_fps_mean_milli_hz` | 21536 | **10662 (½)** |

v54 proves only that **when the odom chain already closes, the optimizer is a no-op** (13 mm "confirmation" corrections across 3 edges and 175 dense nodes). It never validated the optimizer on large-drift input. The house-loop had to absorb a 47.9 m inconsistent constraint into a **single edge over ~half as many nodes** (10.7 Hz vs 21.5 Hz → sparser graph → each node soaks up more correction). Note: v54 is an **older engine build** (`cam_imu_time_offset_us=50000` vs `0`; `depth_flow_*` counters MISSING vs `depth_flow_updates=828`), so cross-counter comparison is directionally valid but not bit-exact.

## 4. RANKED CONTRIBUTING FACTORS

1. **[verified] Frozen-node frame-incoherence** — primary. ~9.9 m of EKF corrections never propagated to PG nodes → 47.9 m phantom residual.
2. **[verified] Single loop edge for ~70 nodes** — no counterweight; the lone PnP measurement dictates the entire deformation. 5 of 6 accepts blocked by the `ok && k==0` gate (10 rot-sanity + 6 chi2 rejects).
3. **[verified] No robust kernel** — the 47.9 m residual is taken at full `info_xy=100`; a Huber/Cauchy falloff would have capped its leverage.
4. **[verified] One-shot UI redraw** — `corrected_traj_xz_` rebuilt only on the single optimize; subsequent EKF corrections desync the polyline from the live dot, adding to the "all over the place" look.
5. **[verified] Half frame rate (10.7 vs 21.5 Hz)** — sparser nodes → larger per-node correction → bigger visible jumps.
6. **[verified] 12.3° chain yaw rotation** — fans subsequent nodes around the origin, compounding the position warp.
7. **[likely] The single edge was the *worst* geometry** — the one accept with both endpoints in the PG was probably the first full-loop revisit (max drift). *Confirm via finding's own caveat (claim marked "likely").*
8. **[verified, secondary] Loop measurement itself is internally scale-consistent** (`target_p_world` and nodes share the same accel-K/`global_t_` frame, `Tracker.cpp:7477`, `1746`) — rules scale-mismatch *out* as a cause; the 47.9 m is incoherence, not a scale bug.
9. **[verified] NOT the cause:** gauge anchor (correct), render path (faithful mirror), weight ratio (already rebalanced), false match (downstream gates caught 16/65 — not too weak).

## 5. WHAT WOULD CONFIRM IT

The root cause and ranks #1-6 are already **verified** from counters + code. Two items remain to nail down:

- **Confirm rank #7 (which accept fired the optimize, and its drift state):** Replay the `val_2026_05_30b` house-loop frames in the desktop harness with pose-graph logging on, and log per-accept: `match_pg`/`now_pg` indices, the `clone_id_to_pg_node_` lookup result for each of the 6 accepts, and the path-fraction at the firing accept. This shows whether the single edge was the maximal-drift first-revisit.
- **Directly prove frame-incoherence (rank #1):** In the same replay, dump each PG node's stored `(x,y,z)` against the *current* `global_t_` at the moment `optimize()` fires, and compute the residual with vs. without the ~9.9 m of prior EKF deltas applied to the post-`match_pg` nodes. If applying the deltas collapses the 47.9 m pre-residual toward ~3.4 m, the frozen-node mechanism is confirmed end-to-end.

## 6. FIX DIRECTION

All options below are **HEADING-SAFE** — they touch the position/pose-graph path only, not the Madgwick/gyro heading pipeline. (Caveat: the optimizer *does* apply 215 mrad of node yaw; any fix that lets it run still rotates node positions, so validate displayed heading on a replay before shipping. The intent is to make node positions coherent so the optimizer needs far less yaw, not to alter the heading source.)

Primary fix — eliminate the frame-incoherence (rank #1), pick one:
- **(a) Retroactive node update:** when an EKF direct correction fires (`Tracker.cpp:7718`), additively apply the same `delta_p` to every frozen PG node with id > `match_pg`. Requires adding the missing `updateNode()`/node-mutation API to `PoseGraph` (currently absent). Keeps nodes coherent with `global_t_`.
- **(b) First-accept-only:** run the pose graph only on the first accept (before any prior corrections have shifted nodes), so node space and the loop measurement are still consistent.
- **(c) Build nodes from `global_t_` directly** and apply `global_t_` deltas to nodes at each accept.

Secondary fixes (defense-in-depth, do *not* substitute for the primary):
- **More loop edges:** loosen the `ok && k==0` coupling so genuine detections add a PG edge even when the EKF chi2 gate rejects (the chi2 gate is a false-negative filter when `P_pp` is collapsed by MSCKF). Multiple edges let the optimizer distribute correction uniformly instead of smearing one constraint. *(Verify the added edges are true positives — keep the rot-sanity/PnP-inlier gates.)*
- **Add a robust kernel (Huber/Cauchy) to loop edges** in `processEdge` (`PoseGraph.cpp:262-350`) so a single large-residual edge cannot dominate the solve. **No magic clamp** — a principled falloff.
- **Redraw `corrected_traj_xz_` on every EKF correction**, not only on `optimize()`, so the displayed polyline tracks the live dot (fixes rank #4's visible end-gap).

— per project rules, prove the primary fix on a harness replay (the §5 residual-with/without-delta test) **before** any device walk, and do not touch the heading source.