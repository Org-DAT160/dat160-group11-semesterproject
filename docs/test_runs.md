# Test runs

Every test run in the simulation is recorded here (README, "Full test runs").
Newest run at the top. Positions are in the `map` frame in metres; the truth is
what scoring prints at start-up.

---

## 2026-11-09 · INT-01 / INT-02 · full runs on all five worlds and tuning

**Setup:** `ros2 launch rescue_team rescue.launch.py variant:=wN leader:=true scoring:=true`
(Gazebo without window), Fast-DDS UDP only, VM with 6 cores and 15 GB RAM. Two runs at
the same time with their own `ROS_DOMAIN_ID` and Gazebo port; real-time factor 0.95–0.99
in every run (`gz stats` every 20 s). Each run gets its own copy of `params.yaml` with the
values in the table; a run ends when scoring stops (10 min) or the mission finishes.
"Min robot gap" is the smallest distance between the two robot centres (odometry).

Score = 600 − t + 100 per accepted marker + 300 for the meeting (§1); 1172 means all five
markers and the meeting after 230 s, 700 means four markers and the meeting, time out.

### Final values, all five worlds (with the corridor fix in the leader)

Final values in `params.yaml`: `viewpoint_spacing` 2.5, `spin_pause` 1.0,
`max_angular_speed` 1.8, `min_sightings` 1, `spin_clearance` 0.90; everything else unchanged
(`max_linear_speed` 0.26). In the tuning table, `spacing`, `ang`, `lin`, `sightings` and
`clearance` are short for `viewpoint_spacing`, `max_angular_speed`, `max_linear_speed`,
`min_sightings` and `spin_clearance`; unnamed values are the old ones.

| Run | World | Changed parameters | Viewpoints | Markers accepted (s after Started) | Meeting done (s) | Finished | Final score | Min robot gap | `peer` |
|---|---|---|---|---|---|---|---|---|---|
| f1_w2 | w2 | final values | 29 + 24 | 4/5: 4 (113), 2 (159), 0 (232), 3 (276) | 124 | no | 700 | 0.97 m | 0 |
| f1_w5 | w5 | final values | 30 + 28 | 5/5: 0 (59), 1 (175), 4 (231), 3 (361), 2 (466) | 287 | 466 s | 935 | 1.01 m | 0 |
| f2_w3 | w3 | final values | 22 + 21 | 4/5: 4 (93), 2 (176), 3 (178), 0 (374) | 124 | no | 700 | 0.52 m | 0 |
| f2_w4 | w4 | final values | 21 + 20 | 5/5: 4 (65), 2 (76), 1 (136), 3 (226), 0 (230) | 85 | 230 s | 1172 | 2.00 m | 0 |
| f3_w1 | w1 | final values | 24 + 25 | 4/5: 0 (70), 4 (102), 2 (351), 1 (498) | 156 | no | 700 | 0.57 m | 0 |
| f3_w1_fast | w1 | final values + lin=0.35 | 24 + 25 | 5/5: 0 (65), 4 (96), 2 (349), 1 (358), 3 (596) | 145 | 596 s | 806 | 0.59 m | 0 |

- **3 of 6 runs complete** (w4 1172, w5 935, w1 806). In the other three exactly **one
  marker was never seen** (w1: 3, w2: 1, w3: 1); the other four and the meeting were done
  by 276–498 s.
- No collision (smallest gap 0.52 m), no `stuck`, no `peer`, no errors in any node.
- With `max_linear_speed` 0.35 the run was complete as well, without the corridor deadlock
  of r3_w1_fast (the leader fix was not triggered: no `peer` result in these runs).

### Tuning runs (w1 first, then all worlds)

| Run | World | Changed parameters | Viewpoints | Markers accepted (s after Started) | Meeting done (s) | Finished | Final score | Min robot gap | `peer` |
|---|---|---|---|---|---|---|---|---|---|
| r1_w1_base | w1 | old values (spacing 1.5, spin_pause 0.5, sightings 3) | 77 + 71 | 3/5: 4 (171), 3 (349), 2 (558) | 232 | no | 600 | 0.53 m | 0 |
| r1_w1_sp25 | w1 | spacing=2.5 | 25 + 29 | 5/5: 0 (69), 4 (84), 2 (317), 1 (441), 3 (576) | 142 | 576 s | 825 | ? | 0 |
| r2_w1_sp30 | w1 | spacing=3.0 | 21 + 17 | 4/5: 4 (85), 2 (312), 1 (346), 0 (399) | 147 | no | 700 | 0.83 m | 0 |
| r2_w1_sp25_p10 | w1 | spacing=2.5 spin_pause=1.0 | 25 + 29 | 5/5: 0 (74), 1 (189), 3 (220), 2 (275), 4 (558) | 589 | 589 s | 818 | 1.59 m | 0 |
| r3_w1_fast | w1 | spacing=2.5 spin_pause=1.0 lin=0.35 | 25 + 29 | 3/5: 0 (70), 4 (87), 2 (314) | 138 | no | 600 | 0.41 m | 10 |
| r3_w1_spin | w1 | spacing=2.5 spin_pause=1.0 spin_speed=1.8 spin_step=1.05 | 25 + 29 | 4/5: 0 (70), 1 (178), 2 (287), 4 (515) | 551 | no | 700 | 1.27 m | 0 |
| r4_w1_best | w1 | spacing=2.5 spin_pause=1.0 | 25 + 29 | 4/5: 0 (78), 1 (205), 2 (276), 4 (567) | 607 | no | 700 | 1.99 m | 0 |
| r4_w1_ang | w1 | spacing=2.5 spin_pause=1.0 ang=1.8 | 25 + 29 | 5/5: 0 (78), 1 (200), 2 (309), 3 (403), 4 (531) | 596 | 596 s | 811 | 1.98 m | 0 |
| r5_w2 | w2 | spacing=2.5 spin_pause=1.0 ang=1.8 | 30 + 28 | 3/5: 0 (119), 4 (156), 1 (448) | 193 | no | 600 | 0.67 m | 0 |
| r5_w3 | w3 | spacing=2.5 spin_pause=1.0 ang=1.8 | 23 + 23 | 4/5: 4 (117), 3 (204), 2 (366), 0 (540) | 149 | no | 700 | 1.41 m | 0 |
| r6_w2 | w2 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 | 30 + 28 | 5/5: 0 (139), 4 (160), 1 (416), 3 (471), 2 (594) | 196 | 594 s | 807 | 0.58 m | 0 |
| r6_w3 | w3 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 | 23 + 23 | 4/5: 2 (114), 4 (117), 3 (189), 0 (232) | 148 | no | 700 | 1.37 m | 0 |
| r7_w4 | w4 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 | 23 + 23 | 4/5: 1 (110), 3 (118), 4 (223), 2 (258) | 267 | no | 700 | 0.68 m | 0 |
| r7_w5 | w5 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 | 31 + 27 | 5/5: 4 (133), 1 (195), 0 (226), 2 (393), 3 (526) | 153 | 526 s | 875 | 0.51 m | 0 |
| r8_w3 | w3 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 clearance=0.9 | 22 + 21 | 4/5: 4 (94), 2 (175), 3 (176), 0 (356) | 124 | no | 700 | 1.17 m | 0 |
| r8_w4 | w4 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 clearance=0.9 | 21 + 20 | 5/5: 4 (66), 2 (73), 1 (133), 3 (227), 0 (230) | 85 | 230 s | 1171 | 2.18 m | 0 |
| r9_w3_sp20 | w3 | spacing=2.0 spin_pause=1.0 ang=1.8 sightings=1 clearance=0.9 | 32 + 34 | 4/5: 4 (102), 3 (190), 2 (243), 0 (258) | 121 | no | 700 | 1.12 m | 0 |
| r9_w1 | w1 | spacing=2.5 spin_pause=1.0 ang=1.8 sightings=1 clearance=0.9 | 24 + 25 | 5/5: 0 (71), 4 (104), 3 (347), 2 (353), 1 (402) | 159 | 402 s | 999 | 0.59 m | 0 |

### Findings
1. **`viewpoint_spacing` 1.5 → 2.5.** With 1.5 m (77 + 71 viewpoints on w1) the robots never
   covered the map: each viewpoint costs a drive and a spin. 3.0 m left a marker unseen.
2. **`min_sightings` 3 → 1 was the largest single improvement.** In r5_w2 and r5_w3 a marker
   was seen once or twice and never reported, and the robots did not come back. Scoring
   has no penalty for a wrong report, the registry retries after a rejection (§7.2), and a
   single sighting was within 1 cm in PER-03. With 1, **every one of the reports in r6–r9 and
   the final runs was accepted at the first attempt.**
3. **`spin_pause` 0.5 → 1.0** gave more sightings per stop (four markers by 275 s instead of
   441 s on w1). `spin_speed` 1.8 with 5 stops (`spin_step` 1.05) missed a marker; kept 1.5 / 0.87.
4. **`spin_clearance` 0.40 → 0.90.** Markers are only detected from 1.0 m (SPK-02). In r6_w3 a
   robot spun 0.96 m from marker 1 and did not see it. Keeping viewpoints 0.9 m from walls
   hardly changes their number (w1: 25 + 28 → 24 + 25).
5. **`max_angular_speed` 1.5 → 1.8** (turning while following a path): 5/5 on w1 against 4/5.
6. **`max_linear_speed` stays 0.26.** In r3_w1_fast (0.35) the robots met in the corridor at
   the top of w1 and deadlocked: both stop on a conflict and neither drives away, and after
   `peer` the leader gave tb3_1 five viewpoints in a row behind tb3_0 (about 65 s lost).
   Fixed in the leader: after a `peer` result, that robot's travel cost is computed with a
   disc around the other robot blocked, so it gets a viewpoint on its own side and drives
   away (`blocked_around_peers` in `leader.py`). Not triggered in the final runs.
7. **Remaining problem: one marker is sometimes never seen.** The viewpoints do not consider
   which way a plate faces. w3 marker 1 (yaw 1.60 rad) was only approached from the east and
   south-west, 70–110° off its normal, in four runs; w2 marker 1 and w1 marker 3 were missed in
   some runs and found in others. Detection is also not reliable: in r4_w1_best a robot
   spun for 20 s 1.6 m from marker 4 without a sighting, where r1_w1_sp25 saw it from the same
   viewpoint. Two simulations at once may take CPU from `aruco_node`; single runs should be
   compared. Possible improvements: viewpoints that face the walls (MIS-05), or sending a
   robot back to the area of a marker that was seen but not accepted.

---

## 2026-11-05 · PER-03 · w1 · marker_detector against the scoring truth

**Setup:** `ros2 launch rescue_team rescue.launch.py variant:=w1 leader:=false scoring:=true`,
real-time factor 0.99, Fast-DDS UDP only. Only tb3_0 used.

**Method.** `robot_handler` drives straight to its target until NAV-05, so it cannot
drive through the maze yet. tb3_0 was therefore **placed** with `gz model` 1.5 m in front
of each marker (free cell, line of sight to the plate). Two phases per marker:

- **A, still:** facing the marker for 3 s. This is what the robot sees while it approaches
  a viewpoint.
- **B, spin:** placed 100° to the right of the marker, then a `navigate_to` goal to the same
  point with `spin_on_arrival`. The turn-and-look spin (§6.8) has to bring the marker into
  view at one of its stops.

During both phases all `/found_marker` messages from tb3_0 and the course topic
`/tb3_0/marker_map_pose` (with `/tb3_0/marker_id`) were recorded. Error = 2D distance from
the truth (we report z = 0).

### Our `marker_detector`

| Marker | Truth (scoring) | Viewpoint | A: n | A: median | A: error (median / max) | B: n | B: error (median / max) |
|---|---|---|---|---|---|---|---|
| 0 | (−1.555, 4.465) | (−0.05, 4.46) | 3 | (−1.558, 4.461) | 0.005 / 0.006 | 3 | 0.005 / 0.009 |
| 1 | (2.108, 7.501) | (3.61, 7.50) | 16 | (2.104, 7.498) | 0.005 / 0.008 | 2 | 0.003 / 0.004 |
| 2 | (0.601, 7.622) | (−0.90, 7.62) | 2 | (0.603, 7.625) | 0.003 / 0.003 | 0 | — |
| 3 | (−0.657, −3.116) | (−0.66, −4.62) | 13 | (−0.661, −3.111) | 0.007 / 0.008 | 3 | 0.001 / 0.004 |
| 4 | (4.813, −0.904) | (5.86, −1.97) | 5 | (4.808, −0.906) | 0.005 / 0.007 | 1 | 0.004 / 0.004 |

- **All five medians within 0.007 m** of the truth (limit 0.5 m). Every single sighting
  within 0.009 m. Camera distance 1.43–1.45 m. z ≈ 0.26 (plate centre).
- No sighting was dropped by the retry queue; most needed one or more retries.
- **Hand report accepted:** `ros2 service call /set_marker_position scoring_interfaces/srv/SetMarkerPosition "{marker_id: 4, marker_position: {x: 4.808, y: -0.906, z: 0.0}}"`
  → `accepted=True`.

### The course `marker_map_pose` (for the report)

| Marker | A (still): n | A: error (median / max) | B (spin): n | B: error (median / max) |
|---|---|---|---|---|
| 0 | 31 | 0.008 / 0.017 | 89 | **1.748 / 3.032** |
| 1 | 30 | 0.003 / 0.007 | 89 | **1.297 / 3.020** |
| 2 | 30 | 0.004 / 0.019 | 88 | **1.602 / 3.014** |
| 3 | 30 | 0.004 / 0.008 | 80 | **1.578 / 3.027** |
| 4 | 27 | 0.002 / 0.007 | 89 | **1.940 / 3.004** |

- Standing still, the course node is as accurate as ours.
- **While the robot turns, it is wrong by up to 3 m.** It stores the last marker pose seen
  by the camera and transforms it again every 0.1 s with the *current* camera pose, also
  after the marker has left the view. The position therefore swings around the robot as it
  turns. It also publishes at 10 Hz whether or not a marker is visible, so the sample count
  says nothing about detections.
- Our node transforms with the camera pose at the image time and only publishes on a new
  detection, so turning does not change its positions (B column above).

### Findings
1. **Marker 2 faces the other way.** Its plate is seen from the west (x < 0.6), against the
   model yaw of 0°; markers 0, 1, 3 and 4 face along their model yaw. The direction of a
   plate cannot be taken from the marker yaw (relevant for viewpoints, MIS-05). Marker 2
   hangs at the end of a narrow corridor and is only visible from inside it.
2. **Few detections right after the robot stops.** Facing a marker for a while, ArUco gives
   13–17 detections per second (as in SPK-02), but in the first seconds after the robot is
   placed or stops it gives far fewer (2–16 in 3 s). In the spin, the 0.5 s stops gave 0–3
   sightings per marker, and none for marker 2. Enough for the position (one sighting is
   within 1 cm), but `min_sightings` is 3. **Suggestion:** try `spin_pause` 1.0 s in the
   tuning runs (INT-02) and compare the sightings per stop.
