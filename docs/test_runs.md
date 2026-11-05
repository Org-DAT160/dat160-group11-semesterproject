# Test runs

Every test run in the simulation is recorded here (README, "Full test runs").
Newest run at the top. Positions are in the `map` frame in metres; the truth is
what scoring prints at start-up.

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
