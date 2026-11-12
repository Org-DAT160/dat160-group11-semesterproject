# Summary: what, where, how and why

A simple overview of the whole solution. Every part has four lines: **what** it
does, **where** it is in the code, **how** it works, and **why** it is built that
way. More detail: [SYSTEM_GUIDE.md](SYSTEM_GUIDE.md) and
[ARCHITECTURE.md](../ARCHITECTURE.md).

## The task in three lines
Two TurtleBot3 robots in Gazebo have 10 minutes to find five ArUco markers (three
small fires, one injured person, one big fire) in a building whose blueprint map is
known. Each marker is reported to the organisers' scoring service, and both robots
must meet at the big fire. Score = 600 − seconds + 100 per marker + 300 for the
meeting; the clock only stops when everything is done.

## The idea in three lines
One **leader** plans everything from the blueprint before the robots move: places
to look from (**viewpoints**), which robot goes where, and the meeting. Each robot
only knows how to **drive to a point** safely. A **marker detector** turns what the
cameras see into positions on the map.

```
 Gazebo (robots, sensors) ──camera──► aruco_node ──► marker_detector ──/found_marker──► leader
        ▲   │ scan, odom                                                                │  │
cmd_vel │   ▼                                                                           │  │ report
   robot_handler ×2  ◄──────────────── NavigateTo action (goal / result) ───────────────┘  ▼
                                                                                        scoring
```

---

## Nodes

### Leader: the brain
- **What:** decides everything: when to start, where each robot goes, when to report
  a marker, when to meet, when the mission is done.
- **Where:** `nodes/leader.py` (ROS part), `core/mission.py` (the decisions).
- **How:** a state machine (`WAIT_FOR_SYSTEM → EXPLORE ⇄ MEET → DONE`). ROS callbacks
  become *events*, the state machine returns *commands* (send goal, cancel goal,
  finish). Every goal has an id and a deadline; an expired goal is cancelled.
- **Why:** one node with the whole picture keeps allocation and the meeting simple.
  The state machine has no ROS in it, so every transition is unit-tested.

### Robot handler: the driver (one per robot)
- **What:** drives its robot to a point it is given, safely, and turns around there
  to look if asked.
- **Where:** `nodes/robot_handler.py`, running as `/tb3_0/robot_handler` and
  `/tb3_1/robot_handler` (same code).
- **How:** an action server `navigate_to`: plan → follow → watch the lidar → recover
  if stuck → give way to the other robot → spin. Answers `reached`, `no path`,
  `stuck`, `peer`, `preempted` or `cancelled`.
- **Why:** an action, because driving takes time and the leader must be able to
  replace a goal (for example when the meeting starts).

### Marker detector: the eyes
- **What:** turns ArUco detections into marker positions on the map.
- **Where:** `nodes/marker_detector.py` (one node for both robots).
- **How:** looks up where the camera was *at the moment the image was taken* (TF at
  the image timestamp; waits up to 1 s if that pose has not arrived yet), transforms
  the marker position into the map frame, publishes a `FoundMarker`.
- **Why:** the course's `marker_map_pose` used the camera pose of *now*, so it was up
  to 3 m wrong while the robot turned (measured in PER-03).

---

## Navigation (our own implementation)

### Grids
- **What:** the blueprint as a grid the planner can use.
- **Where:** `core/grid.py`.
- **How:** the 1 cm map (2000 × 2000) is shrunk to 10 cm cells (200 × 200); walls are
  grown by 0.30 m so the robot can be treated as a point.
- **Why:** 100× fewer cells makes planning fast; growing the walls keeps the robot's
  body (0.24 m turning radius) off them.

### Path planning: distance transform
- **What:** the shortest path from the robot to a goal.
- **Where:** `core/distance_transform.py`, `core/path.py`.
- **How:** start at the goal with cost 0 and spread out: every free cell gets the
  cheapest neighbour's cost + the step (0.1 m, or 0.14 m diagonally). Then walk
  downhill from the robot to the goal, and cut out the zig-zag by jumping straight to
  the farthest point that can be seen.
- **Why:** the map never changes, so nothing like D* is needed; the grid is small, so
  random planners (RRT, PRM) give nothing. And one run gives the distance to *every*
  cell, which the leader also uses to rank viewpoints and pick meeting points. A* only
  gives one path.

### Path following
- **What:** turns the path into wheel commands.
- **Where:** `core/controller.py` (from our assignment A3a).
- **How:** if the next point is more than 0.5 rad off, turn on the spot; otherwise
  drive at `min(0.26 m/s, distance)` and steer towards it.
- **Why:** the robot cannot move sideways, so it turns first; slowing down near each
  point means no overshoot.

### Safety stop and stuck recovery
- **What:** stops for anything unexpected, and gets out when stuck.
- **Where:** `core/lidar.py`, `core/path.py` (`ProgressMonitor`), `robot_handler.py`.
- **How:** lidar points on known walls and on the other robot are ignored; anything
  else closer than 0.35 m ahead stops the robot. No progress for 8 s → reverse up to
  0.30 m and plan again, at most 5 times.
- **Why:** paths may run 0.30 m from walls, so without ignoring known walls the robot
  would stop beside every wall.

### Two robots on one map
- **What:** keeps the robots from crashing into or blocking each other.
- **Where:** `core/traffic.py`, used in `robot_handler.py`; a fix in `leader.py`.
- **How:** every plan avoids a 0.6 m disc around the other robot. If the other robot
  is on my next metre of path, both stop; `tb3_0` has right of way, `tb3_1` takes a
  detour or gives up the goal after 10 s. After giving up, the leader gives it a
  viewpoint on its own side.
- **Why:** neither the map nor the planner knows where the other robot is. The last
  rule fixes a corridor deadlock we saw in testing.

---

## Exploration

### Viewpoints: where to stop and look
- **What:** the list of places the robots visit to look for markers.
- **Where:** `core/viewpoints.py`, called once by the leader before the start.
- **How:**
  - **Pass 0, wall coverage:** every free cell next to a wall is a piece of wall
    facing out. For every place the robot can stand, work out which wall pieces it
    sees from the front (1.3–3.0 m away, at most 55° to the side, no wall in between).
    Then keep picking the place that sees the most unseen wall, until little is left.
  - **Pass 1, lattice:** a point every 2.5 m, as a second round from other places.
- **Why:** markers hang flat on walls and are only detected from the front, from 1 to
  3.5 m (measured in SPK-02). With only the lattice, markers in narrow corridors had
  no viewpoint in front of them and were never found. Computed before the first
  movement, so it costs no time on the scoring clock.

### Allocation: which robot goes where
- **What:** gives a free robot its next viewpoint.
- **Where:** `core/allocation.py` (`GreedyAllocator`).
- **How:** the nearest free viewpoint by *driving* distance (from a distance
  transform), reserved so the other robot goes elsewhere.
- **Why:** simple and good enough with two robots; an auction would only add
  messages, since the leader already knows both distances.

### Look around (spin)
- **What:** the robot turns around at each viewpoint.
- **Where:** `robot_handler.py` (`spin`).
- **How:** 6 steps of 50° with a 1 s stop after each (about 9.5 s).
- **Why:** turning without stopping blurs the image and markers are missed (SPK-02).
  50° is less than the camera's 62° view, so the views overlap.

---

## Markers and the meeting

### Reporting
- **What:** sends each marker's position to scoring.
- **Where:** `core/marker_registry.py`, `leader.py`.
- **How:** report as soon as a marker is seen once: the median of all sightings,
  height 0. If rejected, try the median of the closest sightings.
- **Why:** a wrong report costs nothing, and waiting for three sightings made us miss
  markers that were only seen once.

### The big-fire meeting
- **What:** gets both robots within 2 m of the big fire for +300.
- **Where:** `core/meeting.py`, `core/mission.py`.
- **How:** when marker 4 is accepted, the closer robot goes to a point within 1.0 m of
  it, the other to a point within 1.2 m and at least 0.8 m from the first. Both wait 3 s,
  then continue exploring.
- **Why:** 1.2 m + 0.5 m (largest report error) + 0.1 m (stopping error) < 2 m, so it
  always counts. Two points, so the robots do not block each other in a narrow alcove.

---

## Communication

| Pattern | Used for | Why |
|---|---|---|
| Topic | sensors, `cmd_vel`, `/found_marker`, `/map` | streams, no answer needed |
| Service | reporting to scoring | one question, one answer |
| Action | leader → robot handlers | long task that can be replaced or cancelled |

The robots never talk to each other directly: the leader coordinates, and each robot
handler reads the other robot's odometry for the two-robot rules. Messages go over
UDP, because with the default shared memory the camera images arrived several times
slower (3.7 instead of 21–24 per second, SPK-01).

## Code and tests
- **Where:** `rescue_team/core/` (all algorithms, pure Python + NumPy, no ROS),
  `rescue_team/nodes/` (the three ROS nodes), `config/params.yaml` (every value),
  `launch/rescue.launch.py`.
- **How tested:** 289 unit tests on `core/`; measurements of the camera, odometry and
  scoring before designing (SPK-01, SPK-02); full 10-minute runs on all five worlds.
- **Why this split:** the logic can be tested in seconds without Gazebo, and the ROS
  nodes stay thin.

## From 600 to 1071 points

| Stage | What changed | Result |
|---|---|---|
| First full run (INT-01) | everything working together | 3/5 markers, 600 |
| Tuning (INT-02) | fewer viewpoints, longer pauses, report after one sighting | 4/5 on three worlds, about 840 average |
| Wall coverage | viewpoints chosen by what they see | **5/5 + meeting on all five worlds, 1071 average** |
