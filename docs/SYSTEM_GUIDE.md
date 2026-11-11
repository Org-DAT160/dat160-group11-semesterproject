# System guide: how our Search & Rescue solution works

A read-through for the whole group before the competition and the oral exam. It
explains what every part does, why it is built that way, and where to find it in
the code. The full specification is [ARCHITECTURE.md](../ARCHITECTURE.md) (section
numbers § below refer to it), the module and function list is
[CODE_STRUCTURE.md](CODE_STRUCTURE.md), and all measurements are in
[test_runs.md](test_runs.md).

---

## 1. Summary

Two TurtleBot3 robots search a known building (the blueprint map) for five ArUco
markers in 10 minutes, report each marker's position to the organisers' scoring
service, and meet at the big fire (marker 4).

- **Centralised.** One `leader` node makes every decision. Each robot runs a
  `robot_handler` that only knows how to drive to a point. One `marker_detector`
  turns camera detections into map positions.
- **Exploration.** Before the robots move, the leader computes **viewpoints**:
  places from which every wall can be seen from the front. The robots visit them
  nearest-first and turn around at each one to look.
- **Navigation (our own, R7).** Paths are planned on the blueprint with a
  **distance transform** (Corke ch. 5), followed with a go-to-point controller
  (A3a), protected by a lidar safety stop, stuck recovery and right-of-way rules
  between the two robots.
- **Markers.** `marker_detector` transforms each ArUco detection into the `map`
  frame with the camera pose *at the image time*. The leader reports the median
  of the sightings and retries if a report is rejected.
- **Meeting.** When the big fire is accepted, both robots drive to two points
  within about 1 m of it and wait 3 s, then continue exploring.
- **Result.** In the final test runs we found all five markers and held the meeting
  on all five worlds, with an average score of 1071 (score = 1400 − seconds when
  everything is done).

```
             /map            odom, scan                camera
map_server ───────┐      ┌───────────── Gazebo ────────────► aruco_node (per robot)
                  ▼      ▼                 ▲                      │ /tb3_N/aruco_markers
            ┌───────────────────┐  cmd_vel │                      ▼
            │ robot_handler ×2  │──────────┘             ┌─────────────────┐
            │ (action server)   │                         │ marker_detector │
            └───────────────────┘                         └─────────────────┘
                  ▲   NavigateTo action                          │ /found_marker
                  │   (goal, result)                             ▼
            ┌──────────────────────────────────────────────────────────────┐
            │ leader: Mission state machine, viewpoints, allocation,       │
            │         marker register, meeting                             │
            └──────────────────────────────────────────────────────────────┘
                                      │ SetMarkerPosition service
                                      ▼
                                   scoring (organisers)
```

---

## 2. One run, from launch to the end

1. **Launch.** `rescue.launch.py` starts Gazebo, `map_server` and both robots, or
   only our nodes with `start_world:=false`. It also starts our three nodes with
   `params.yaml` and Fast-DDS over UDP (§7.1: with shared memory the camera images
   arrived only 0.5–4 times per second).
2. **Waiting for the system (§11).** The leader checks that the simulation clock
   runs, `/map` has arrived, both robots publish odometry, the viewpoints are
   computed, the scoring service exists and both `navigate_to` action servers
   exist. Every 5 s it logs what is still missing. All conditions must hold for 2 s.
3. **Before the clock starts.** Scoring's clock only starts when a robot moves
   (§2), so everything heavy is done before the first goal: grids, wall distance
   and viewpoints (about 1–2 s).
4. **`Started`.** The leader gives each robot the cheapest free viewpoint. The
   robot plans a path, drives, and turns around in 6 steps of about 50° with a
   1 s pause after each step.
5. **Markers.** While the robots drive and turn, `aruco_node` sees markers.
   `marker_detector` publishes a `FoundMarker` per sighting. The leader stores it
   and reports the marker on its next tick (`min_sightings` = 1).
6. **Big fire.** When scoring accepts marker 4, the `Mission` computes meeting
   points A and B. Both robots get new goals at once (replacing their viewpoints),
   wait 3 s together, and explore again.
7. **Passes.** When every viewpoint of pass 0 has been visited, the leader loads
   pass 1 (a lattice between the first points), then pass 0 again, and so on.
8. **End.** When all five markers are accepted and the meeting is done, the leader
   cancels all goals and logs `MISSION FINISHED`. Scoring stops its clock.

---

## 3. The nodes

### 3.1 `leader` (one, global): `nodes/leader.py` + `core/mission.py`

The leader is an **adapter** around the `Mission` class. `Mission` is plain Python
with no ROS and no clock: it receives **events** and returns **commands**.

```
ROS callback ──► Event ──► Mission.handle(event) ──► [Command, ...] ──► ROS call
```

| Events in | Commands out |
|---|---|
| `Started`, `GoalFinished`, `MarkerAccepted`, `Tick` (2 Hz) | `SendGoal`, `CancelGoal`, `Finish` |

Why this split: the whole mission logic can be unit-tested without Gazebo (every
transition has a test), and the ROS code stays thin.

**States (§5):** `WAIT_FOR_SYSTEM → EXPLORE ⇄ MEET → DONE`.

**Goal ids and deadlines (§5.1).** Every `SendGoal` gets a new `goal_id` and a
deadline of `20 s + 3 · travel distance / 0.26 m/s (+ spin time)`. A result with
an old `goal_id` is ignored (it belongs to a goal that was replaced). A goal past
its deadline is cancelled and handled as a failure, so a lost message or a robot
that hangs can never stop the mission.

**Concurrency.** The leader is single-threaded. It never blocks waiting for a
result: action goals use `send_goal_async` and service calls `call_async`, each
with a done callback. Blocking inside a callback would deadlock the node, because
the executor that should deliver the answer is busy running that callback.

### 3.2 `robot_handler` (one per robot): `nodes/robot_handler.py`

The same code runs as `/tb3_0/robot_handler` and `/tb3_1/robot_handler`. It uses
relative topic names, so the namespace picks the robot, and it reads the other
robot's odometry from the `robots` parameter. It offers one **action**,
`navigate_to` (`NavigateTo.action`: target, `spin_on_arrival` → success, message).

For each goal:
1. **Plan** with the distance transform (§4.1 below), with a disc around the other
   robot.
2. **Follow** the path with the go-to-point controller every 0.05 s.
3. **Safety stop** if the lidar sees something unexpected ahead.
4. **Progress check**: no progress for 8 s → reverse a little and plan again (up
   to 5 times).
5. **Two-robot rules** on a conflict with the other robot.
6. **Spin** at the target if requested.

The result message is one of `reached`, `no path`, `stuck`, `peer`, `preempted`,
`cancelled`.

**Concurrency.** The execute callback runs for the whole goal, so the node uses a
`MultiThreadedExecutor(4)` with a `ReentrantCallbackGroup`. That way `odom` and
`scan` callbacks keep running while a goal is driving. A new goal replaces the old
one: the control loop checks before every command that its goal is still the
active one.

### 3.3 `marker_detector` (one, global): `nodes/marker_detector.py`

1. Subscribes to `/tb3_N/aruco_markers` for both robots.
2. For every marker with ID 0–4, looks up `map ← tb3_N/camera_rgb_optical_frame`
   **at the image timestamp**. The pose often arrives a few ms after the image, so
   messages wait in a queue and are retried every 20 ms for up to 1 s.
3. Transforms the marker position with NumPy, drops sightings farther than 3 m,
   publishes `FoundMarker` (id, map position, distance, robot).

Why our own node instead of the course `marker_map_pose`: the course node
re-transforms the *last* camera pose with the *current* robot pose every 0.1 s.
While the robot turns, its positions were wrong by up to 3 m (PER-03,
test_runs.md). Ours uses the pose at the image time and only publishes on a real
detection. It is one global node because a namespaced Python TF listener in Foxy
receives no transforms (§2).

---

## 4. Navigation (our own implementation, R7)

### 4.1 Grids and the distance transform (§6.1, §6.2)

- `/map` is 2000 × 2000 cells at 1 cm. We **downsample** to 10 cm (a coarse cell is
  a wall if any fine cell is), giving 200 × 200 cells: the **static grid**.
- We **inflate** walls by 0.30 m: the **planning grid**. In it the robot can be
  treated as a point (configuration space). The robot sweeps about 0.24 m when it
  turns; the rest is margin.
- **Distance transform** (Corke ch. 5): from one or more source cells, compute the
  travel distance `D` to every cell. Sources get 0, every other free cell
  `D(c) = min over its 8 neighbours n of D(n) + step` (step 0.1 m or 0.141 m
  diagonally). A diagonal step is only allowed when both side cells are free, so
  paths never cut a wall corner. Implemented as repeated relaxation with NumPy
  array shifts until nothing changes (`core/distance_transform.py`, `cost_field`).
- **Path:** put the field's source at the goal and walk downhill from the robot
  (steepest descent, `descend`). Then remove zig-zag: from each point jump to the
  farthest later point in a straight line of sight (`simplify_path`), so only
  corner points remain.

**One function, five uses:** path to a goal, travel cost to every viewpoint
(allocation), reachable viewpoints, distance to the nearest wall, and the meeting
points.

**Why a distance transform** (often asked):
- The map never changes during a run, so D* (plan repair) gives nothing.
- The grid is small (200 × 200), so sampling planners (PRM, RRT) give no advantage
  and produce random paths that need smoothing.
- The TurtleBot turns on the spot, so lattice planners' turning-radius handling is
  not needed.
- A* gives one path between two cells. One distance transform gives the distance
  from the sources to **every** cell, which the leader needs for allocation and
  meeting points anyway.

**Why not wall following or Bug2** (§6.9): they are for robots that do not know
the map. Here the map is known, so the problem is *coverage*. Wall following never
reaches walls that are not connected to the outer wall, and the camera looks along
the wall instead of at it. Bug2 needs a known goal and ignores the blueprint.

### 4.2 Path following (§6.4): `core/controller.py`

Go-to-point controller from A3a (Corke ch. 4). With distance `d` and heading error
`e` to the next path point:

```
|e| > 0.5 rad:  v = 0,                    ω = 1.5 · e      (turn in place)
otherwise:      v = min(0.26, 1.0 · d),   ω = 1.5 · e      (drive, slow down near the point)
```

`ω` is limited to ±1.8 rad/s. Intermediate points count as reached within 5 cm,
the goal within 10 cm.

### 4.3 Reactive layer (§6.5)

- **Lidar safety stop.** Each scan becomes map-frame points. Points on known walls
  and points near the other robot are dropped (otherwise the robot would stop
  beside every wall, since paths are allowed 0.30 m from walls). If a remaining
  point is less than 0.35 m ahead and within ±0.20 m sideways, the robot stops and
  waits.
- **Stuck recovery.** `ProgressMonitor`: the remaining path must shrink by 5 cm
  within 8 s. If not, the robot stops, reverses up to 0.30 m (only if the rear
  sector and the other robot allow it) and plans again. After 5 tries the goal ends
  with `stuck`.
- Nothing is ever added to the grid: the worlds have no obstacles that the
  blueprint does not show, and a temporary obstacle could close a corridor.

### 4.4 Two robots on one map (§6.6): `core/traffic.py`

1. **Disc in every plan.** Each plan blocks a 0.60 m disc around the other robot.
2. **Conflict.** A point on my next 1.0 m of path is within 0.50 m of the other
   robot **and** closer to it than I am now. Driving away is therefore always
   allowed.
3. **Right of way.** On a conflict both stop. `tb3_0` (first in `robots`) has right
   of way and waits. `tb3_1` plans a detour around the disc, or waits.
4. **Timeout.** After 10 s, the robot without right of way ends its goal with
   `peer`, and the leader gives it another viewpoint. After a `peer`, the leader
   computes that robot's travel costs *around* the other robot, so the next
   viewpoint is on its own side (fix for a corridor deadlock found in INT-02).

### 4.5 Spin at viewpoints (§6.8)

Continuous turning blurs the camera image (SPK-02: 0–8 detections per turn at
≥ 1 rad/s). So the robot **turns and looks**: 6 steps of 0.87 rad (50°, less than
the 62° field of view, so the views overlap) at 1.5 rad/s, with 1.0 s standstill
after each step. The total angle is 5.2 rad (2π − 62°), because the camera already
saw ahead while driving there.

---

## 5. Exploration (§8)

### 5.1 Viewpoints: `core/viewpoints.py`

**Pass 0: wall coverage** (`generate_coverage_viewpoints`).
1. Every free cell next to a wall is a **wall face**, with a normal pointing away
   from the wall. Markers hang on such faces.
2. Every cell the robot can stand in is a candidate: free in the planning grid, at
   least 0.5 m from a wall, and reachable from the start positions.
3. A candidate **sees** a face if it is 1.3–3.0 m away, at most 55° off the face's
   normal, and no wall cell lies on the straight line between them.
4. **Greedy set cover:** take the candidate that sees the most faces not seen yet;
   repeat until no candidate adds at least 8 faces (0.8 m of wall).

**Pass 1: lattice** (`generate_viewpoints`): points every 2.5 m, moved to a nearby
cell 0.9 m from walls, kept if reachable and at most 1.5 m from a wall. It looks
from other places in a second round.

Why the numbers: SPK-02 measured detection from 1.0 m (closer, the top of the plate
leaves the image) to 3.5 m, and up to 75° off the normal. 1.3 m and 55° leave a
margin. 3.0 m is also `marker_max_range`, where the position error is about 3 cm.

Why coverage instead of only the lattice: with the lattice, half of the INT-02 runs
missed one marker. An offline check showed that w2 marker 1 and w3 marker 1 had
**no** viewpoint in front of them, because both hang in corridors under 1.8 m wide
and the lattice needs 0.9 m to *every* wall. Coverage viewpoints are chosen by what
they see, so they also land in narrow corridors. Results: test_runs.md.

### 5.2 Allocation: `core/allocation.py`

`GreedyAllocator`: when a robot is free, it gets the free viewpoint with the
shortest **travel** distance (from a distance transform with the robot as source,
cached while the robot stays in the same cell). The viewpoint is reserved, so the
other robot goes elsewhere. On a failure the viewpoint is offered once to the other
robot, then dropped.

Why not a market/auction: the leader already knows both robots' travel distances,
so bidding would only add messages.

---

## 6. Marker reporting (§7.2): `core/marker_registry.py`

- Each sighting is stored per marker ID.
- **First report** as soon as there is one sighting (`min_sightings` 1): the
  **median** x, y of all sightings, with z = 0 (the marker model's origin is on the
  floor; scoring accepts within 0.5 m in 3D).
- **Rejected** → report the median of the closer half of the sightings (closer is
  more accurate). Rejected again → wait for new sightings.
- A report without an answer after 5 s is sent again.
- A wrong report costs nothing, which is why `min_sightings` is 1: in INT-02,
  markers were often seen only once or twice, and with 3 they were never reported.
  Every report since was accepted at the first attempt.

---

## 7. Big-fire meeting (§9): `core/meeting.py`

- Scoring gives +300 if, at any time after marker 4 is accepted, **both** robots are
  within 2 m of it. It checks every 0.5 s.
- **Point A:** for the robot that is closer by travel distance, the reachable cell
  within 1.0 m of the reported position with the shortest travel distance.
- **Point B:** for the other robot, a reachable cell within 1.2 m, at least 0.8 m
  from A. In a narrow alcove this puts the second robot at the opening.
- **Bound:** report error < 0.5 m, point ≤ 1.2 m from the report, robot ≤ 0.1 m from
  its point, so the robot is < 1.8 m from the true marker, inside 2 m.
- Both wait 3 s. If a robot cannot reach its point, the point is excluded and new
  points are computed (up to 3 times); later the meeting is retried.

---

## 8. Communication patterns (R8)

| Pattern | Used for | Why |
|---|---|---|
| **Topic** | sensors (`scan`, `odom`, camera), `cmd_vel`, `/found_marker`, `/map` | Continuous streams, many publishers to one subscriber, no answer needed. `/map` uses transient-local QoS because `map_server` publishes it only once. |
| **Service** | reporting to scoring (`/set_marker_position`) | One short question, one answer (`accepted`). |
| **Action** | leader → `robot_handler` (`navigate_to`) | Long task with a result, which can be replaced or cancelled, for example when the meeting starts. |

**Centralised versus decentralised:** one node has the whole picture, which keeps
allocation and the meeting simple and testable. Decentralised claiming of
viewpoints was considered, but is harder to test and gives no benefit with two
robots on a reliable simulated network. The cost is a single point of failure,
which is acceptable in simulation.

---

## 9. Code structure and tests (§12)

```
rescue_interfaces/        NavigateTo.action, FoundMarker.msg
rescue_team/
  rescue_team/core/       algorithms and decisions, pure Python + NumPy, no ROS
  rescue_team/nodes/      thin ROS 2 adapters: leader, robot_handler, marker_detector
  launch/rescue.launch.py world, robots and our nodes
  config/params.yaml      every tunable value
  test/                   289 pytest unit tests, flake8, pep257
```

- `core/` never imports rclpy and never reads a clock (time is passed in as `now`),
  so it is tested with `pytest` without Gazebo.
- No robot name appears in the code; everything comes from the `robots` parameter.
- Python 3.8 (Foxy): `typing.List`/`Tuple`, no `match`, no SciPy.

---

## 10. Numbers worth knowing

| What | Value | Where from |
|---|---|---|
| Grid | 0.10 m cells, 200 × 200, walls inflated 0.30 m | §6.1 |
| Speed | 0.26 m/s, turns up to 1.8 rad/s | TurtleBot3 limit; INT-02 |
| Spin | 6 × 50° at 1.5 rad/s, 1.0 s pause, about 9.5 s | SPK-02, INT-02 |
| Camera | 62° wide, markers detected 1.0–3.5 m, up to 75° off the normal | SPK-02 |
| Detection without UDP | 0.5–4 images/s instead of 15–20 | SPK-02 |
| Scoring | accept within 0.5 m in 3D; meeting within 2 m; clock starts at the first movement | §2, SPK-01 |
| Score | 600 − t + 100 per marker + 300 meeting = **1400 − t** when everything is done | §1 |
| Final runs | 5/5 + meeting on w1–w5: 937, 1070, 1185, 1125, 1037 | test_runs.md |

---

## 11. Questions to be ready for

- **How do the robots communicate?** They do not talk to each other directly. The
  leader sends goals over an action, each robot reads the other's odometry topic
  for the two-robot rules, and sightings go through a topic to the leader. See §8.
- **Which navigation algorithm runs, and what did you implement?** Distance
  transform planning, line-of-sight path simplification, a go-to-point controller,
  safety stop, stuck recovery, two-robot rules, viewpoint generation, allocation
  and meeting points. All of it is in `core/` and written for this project. From
  the course we reused node skeletons, the action server/client pattern and the
  launch file structure; from our own assignments the A3a go-to-point controller
  and the A1/A3 lidar sector logic (CODE_STRUCTURE.md, "Where the code comes
  from").
- **Why not Nav2?** R7 requires our own implementation, and the map is static and
  small, so a distance transform does the job.
- **What happens if a message is lost or a robot hangs?** The goal deadline expires,
  the leader cancels the goal and gives a new one.
- **What if both robots meet in a corridor?** §4.4: the robot with right of way
  waits; the other one detours, or gives up its goal after 10 s and gets a
  viewpoint on its own side.
- **Why `min_sightings` 1?** Wrong reports cost nothing, and waiting for more
  sightings made us miss markers.
- **Why turn-and-look?** Continuous turning blurs the image (SPK-02).
- **Why report z = 0?** The marker model's origin is on the floor; the plate centre
  is at 0.25 m, which would use half of the 0.5 m tolerance.
- **What would you improve?** Turn only towards the walls a coverage viewpoint
  covers instead of a full turn (time), and a better visibility model for markers
  whose front cannot be reached.
- **How did you test?** 289 unit tests on `core/`; spikes SPK-01/02 measured the
  camera, odometry and scoring; full 10-minute runs on all worlds, two at a time
  with their own ROS domain and Gazebo port (test_runs.md).

---

## 12. Running it

See the README, "Running". In short:

```bash
# Build (only the packages the system needs, as in the README)
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
colcon build --packages-select multi_robot_challenge_23 scoring scoring_interfaces \
  ros2_aruco ros2_aruco_interfaces rescue_interfaces rescue_team
source install/setup.bash

# Everything: world, robots, our nodes
ros2 launch rescue_team rescue.launch.py variant:=w1
ros2 run scoring scoring          # second terminal, after the robots have spawned

# If the organisers start the world themselves
ros2 launch rescue_team rescue.launch.py start_world:=false
```

The competition-day checklist is [COMPETITION_CHECKLIST.md](COMPETITION_CHECKLIST.md).
