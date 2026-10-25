# DAT160 Search & Rescue — Group 11

A team of two TurtleBot3 robots that searches a known maze for ArUco markers at
unknown positions, reports fires and an injured person, and meets at the big
fire. Semester project in DAT160 Intelligent Robots, Western Norway University
of Applied Sciences (HVL), autumn 2026.

- **Design and interfaces:** [ARCHITECTURE.md](ARCHITECTURE.md)
- **Code structure:** [docs/CODE_STRUCTURE.md](docs/CODE_STRUCTURE.md)
- **Work plan:** [docs/ISSUES_OVERVIEW.md](docs/ISSUES_OVERVIEW.md)
- **Work logs:** [docs/worklog/](docs/worklog/)

## Team

| Name | Owner | Area |
|---|---|---|
| _Name_ | A | Navigation algorithms (grid, distance transform, paths, rules for two robots, marker register, viewpoints, allocation) |
| _Name_ | B | Robot node and perception (geometry, lidar, controller, `robot_handler`, `marker_detector`, launch file, parameters) |
| _Name_ | C | Mission and leader (`Mission` state machine, `leader`, reporting, big-fire meeting) |

---

## Requirements

| Component | Version |
|---|---|
| Ubuntu | 20.04 |
| ROS 2 | Foxy |
| Gazebo | Classic 11 |
| Python | 3.8 |

Course packages from
[`frdedynamics/ros2_students_25`](https://github.com/frdedynamics/ros2_students_25):
`multi_robot_challenge_23`, `scoring`, `scoring_interfaces`, `ros2_aruco`,
`ros2_aruco_interfaces`.

---

## Installation

### 1. System dependencies

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions \
  ros-foxy-gazebo-ros-pkgs ros-foxy-turtlebot3* \
  ros-foxy-nav2-map-server ros-foxy-nav2-lifecycle-manager \
  ros-foxy-tf-transformations ros-foxy-tf2-geometry-msgs \
  ros-foxy-teleop-twist-keyboard
pip3 install transforms3d opencv-contrib-python
```

ROS 2 Foxy itself is assumed to be installed in `/opt/ros/foxy`, as in the
course VM.

The course `aruco_node` needs an `opencv-contrib-python` version that provides
`getPredefinedDictionary`, `DetectorParameters()`, `detectMarkers` and
`estimatePoseSingleMarkers`. Version verified in the course VM: `opencv-contrib-python==4.8.0.76`.

### 2. Workspace

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/frdedynamics/ros2_students_25.git
git clone https://github.com/Org-DAT160/dat160-group11-semesterproject.git dat160_group11
```

### 3. Course files

The scoring node needs the `gazebo_ros_state` plugin, which is only present in
the world files of the scoring package. Copy them over the challenge world files,
and install the Gazebo models:

```bash
cd ~/ros2_ws/src/ros2_students_25
cp multi_robot_scoring/worlds/*.world multi_robot_challenge_23/worlds/
mkdir -p ~/.gazebo/models
cp -r multi_robot_challenge_23/models/* ~/.gazebo/models/
```

Instead of copying the models, the folder can be added to `GAZEBO_MODEL_PATH`.

### 4. Build

```bash
source /opt/ros/foxy/setup.bash
cd ~/ros2_ws
colcon build --packages-select \
  multi_robot_challenge_23 scoring scoring_interfaces \
  ros2_aruco ros2_aruco_interfaces \
  rescue_interfaces rescue_team
source install/setup.bash
```

Only the packages the system needs are built: the course repository also
contains packages for other exercises (for example `turtlebot3_manipulation`)
with dependencies that are not installed in the VM. The build does not use
`--symlink-install`, so it behaves the same on every machine.

Add `source /opt/ros/foxy/setup.bash` and `source ~/ros2_ws/install/setup.bash`
to `~/.bashrc` to avoid repeating them in every terminal. After a change in our
code, `colcon build --packages-select rescue_team` is enough.

### 5. Fast-DDS over UDP (recommended for development)

By default, Fast-DDS (the ROS 2 Foxy middleware) also passes messages through
shared-memory files in `/dev/shm`. Processes that are killed leave these files
behind, and after many runs topics go silent: they are listed, but no data
arrives. On the course VM the camera also ran at only 3.7 Hz with the default,
and at 21–24 Hz over UDP (SPK-01). Add this line to `~/.bashrc`:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/ros2_ws/src/dat160_group11/rescue_team/config/fastdds_udp_only.xml
```

This only changes your own machine. Our code does not depend on it and works
with the default setting as well (ARCHITECTURE.md §7.1), which is what the
competition PC is likely to use.

---

## Running

### Before every run

Stop everything from the previous run, so that no old node is left in the ROS
graph. An old scoring node answers every report with "not accepted".

```bash
# Ctrl+C in every terminal of the previous run (launch file and scoring), then:
pkill -f gzserver; pkill -f gzclient
ros2 node list    # after a few seconds this should print nothing
```

In a shared lab network, keep ROS traffic on the local machine:

```bash
export ROS_LOCALHOST_ONLY=1           # also add it to ~/.bashrc
```

### Full system

```bash
# Terminal 1: world, map, robots, ArUco detection and our nodes
ros2 launch rescue_team rescue.launch.py variant:=w1

# Terminal 2: scoring, started once both robots have spawned
ros2 run scoring scoring
```

The robots start moving when the leader has checked that everything is ready.
Until then it logs every 5 s what it is still waiting for. The Gazebo window is
off by default because scoring measures wall-clock time; add `gui:=true` to
watch the run.

| Launch argument | Values | Default | Meaning |
|---|---|---|---|
| `variant` | `w1` … `w5` | `w1` | World variant: world file, map and spawn poses |
| `start_world` | `true`, `false` | `true` | Start Gazebo, the map server and both robots. Set to `false` if the world is started by another launch file, for example the organisers' own. |
| `scoring` | `true`, `false` | `false` | Also start the scoring node, after a delay so that it starts after the robots have spawned. Leave `false` in the competition, where the organisers run scoring. |
| `rviz` | `true`, `false` | `false` | Start RViz |
| `gui` | `true`, `false` | `false` | Show the Gazebo window. It is off by default because it slows the simulation, and scoring measures wall-clock time. |
| `leader` | `true`, `false` | `true` | Start the `leader`. With `false`, the robots only drive to goals sent by hand. |
| `use_sim_time` | `true`, `false` | `true` | Passed to all our nodes. Leave `true`. |
| `udp_dds` | `true`, `false` | `true` | Fast-DDS over UDP only for every process this file starts (Gazebo, cameras, `aruco_node`, our nodes). With shared memory, camera images reached `aruco_node` only 0.5–4 times per second (SPK-02). |

### Only our nodes (world already running)

```bash
ros2 launch rescue_team rescue.launch.py start_world:=false
```

### Hand-sent goals (development)

```bash
ros2 launch rescue_team rescue.launch.py variant:=w1 leader:=false

# Drive tb3_0 to a point and rotate there
ros2 action send_goal /tb3_0/navigate_to rescue_interfaces/action/NavigateTo \
  "{target: {x: 2.0, y: 1.0, z: 0.0}, spin_on_arrival: true}" --feedback

# Watch the marker sightings
ros2 topic echo /found_marker

# Report a marker to scoring by hand
ros2 service call /set_marker_position scoring_interfaces/srv/SetMarkerPosition \
  "{marker_id: 0, marker_position: {x: 1.0, y: 2.0, z: 0.0}}"
```

### Single nodes (development)

```bash
PARAMS=~/ros2_ws/src/dat160_group11/rescue_team/config/params.yaml
ros2 run rescue_team robot_handler --ros-args -r __ns:=/tb3_0 --params-file $PARAMS -p use_sim_time:=true
ros2 run rescue_team marker_detector --ros-args --params-file $PARAMS -p use_sim_time:=true
ros2 run rescue_team leader --ros-args --params-file $PARAMS -p use_sim_time:=true
```

`marker_detector` and `leader` run without a namespace.

---

## Testing

Algorithm code in `rescue_team/rescue_team/core/` has no ROS dependencies and
can be tested on any machine with Python 3.8, `numpy`, `pyyaml`, `pillow` and
`pytest`. The two linter tests need the ROS 2 lint packages, so they are skipped
outside the VM:

```bash
cd rescue_team
python3 -m pytest test/ -v --ignore=test/test_flake8.py --ignore=test/test_pep257.py
```

In the VM, with ROS 2 sourced, all tests run:

```bash
cd rescue_team
python3 -m pytest test/ -v
```

Inside the ROS workspace:

```bash
colcon test --packages-select rescue_team && colcon test-result --verbose
```

### Full test runs

Every full run is recorded in `docs/test_runs.md`.

1. Stop everything from the previous run (see "Before every run"). Gazebo's
   "Reset World" is not used: the world files contain two different poses per
   marker, and a reset can move the markers away from the positions scoring
   uses. Our nodes are restarted too.
2. Start the launch file without the Gazebo window (the default).
3. Start scoring after the robots have spawned, and copy the marker positions
   it prints at start-up into the log. These are the ground truth.
4. Record the real-time factor (`gz stats`). Scoring measures time with the wall
   clock, so a slow simulation costs points.
5. Record score, time, markers found, meeting and failures.

---

## Repository structure

```
.
├── README.md
├── ARCHITECTURE.md
├── docs/
│   ├── CODE_STRUCTURE.md   # classes, methods and where the code comes from
│   ├── ISSUES_OVERVIEW.md  # issue plan, dependencies and milestones
│   ├── test_runs.md        # log of full test runs
│   ├── worklog/            # one work log per group member
│   └── report/             # figures and drafts for the report
├── tools/
│   ├── create_issues.py    # creates GitHub issues from issues.csv (gh CLI)
│   └── issues.csv
├── rescue_interfaces/      # custom action and message types (ament_cmake)
└── rescue_team/            # all nodes and algorithms (ament_python)
```

See [docs/CODE_STRUCTURE.md](docs/CODE_STRUCTURE.md) for the role of each file.

---

## Development workflow

- `main` is protected. Work on a branch and open a pull request with one
  reviewer. Changes to a file owned by someone else are reviewed by its owner
  (see [ARCHITECTURE.md §14](ARCHITECTURE.md#14-ownership-and-workflow)).
- `rescue_interfaces/` is frozen after milestone M0.
- Code issues are labelled `impl:own` or `impl:ai-allowed`:
  - `impl:own`: core of the navigation algorithm, implemented by the issue owner
    without AI code generation. AI tools may be used for explanations and review.
  - `impl:ai-allowed`: AI tools may be used for code generation; the owner
    reviews the code and can explain it.
- Code must run on **Python 3.8** with NumPy 1.17. Type aliases use `typing`
  (`Tuple`, `List`, `Dict`, `Optional`); no `match` statements. Robot code only
  uses `numpy`, `opencv` and ROS 2 packages.
- Use Unix line endings: `git config --global core.autocrlf input` on Windows.
- Tunable values belong in `config/params.yaml`, never in code.
- Local experiments and helper scripts that are not part of the delivery go in
  `local/`, which is excluded from git.
- Each member records their work in `docs/worklog/<name>.md`.

---

## Troubleshooting

| Problem | Cause and fix |
|---|---|
| Scoring prints `service not available, waiting again...` forever | World files without the `gazebo_ros_state` plugin. Repeat installation step 3. |
| Gazebo shows no markers or walls | Models missing from `~/.gazebo/models`. Repeat installation step 3. |
| Correct reports are answered with "not accepted" | An old scoring node from a previous run is still running, or the world was reset. Stop everything (see "Before every run") and start again. |
| The robots never start moving | The leader logs the conditions it is still waiting for every 5 s: simulation clock, `/map`, odometry, the scoring service and both `navigate_to` action servers. Start whatever is missing. |
| The leader logs an error about `aruco_markers` or `camera/image_raw` | The ArUco detection or the camera of that robot is not running. Check the output of the launch file. |
| No messages on `/found_marker` while a marker is in view | Check that `/tb3_0/aruco_markers` publishes (`ros2 topic echo`) and look for transform failures in the `marker_detector` log. |
| `AttributeError` from `cv2.aruco` | OpenCV version incompatible with the course `aruco_node`, which uses `getPredefinedDictionary`, `DetectorParameters()`, `detectMarkers` and `estimatePoseSingleMarkers`. Install an `opencv-contrib-python` version that provides all four, or use the updated package [LauEls/ros2_aruco](https://github.com/LauEls/ros2_aruco) recommended on the competition page. |
| Markers visible in Gazebo but not detected | Lighting. Add or remove lights above the markers in the world file (see the course page). |
| Scoring clock starts immediately | Scoring was started before the robots published odometry. Start scoring after the robots have spawned. |
| Nodes from other groups appear in `ros2 node list` | Set `ROS_LOCALHOST_ONLY=1` (see "Before every run"). |
| Warning about two nodes named `marker_recognition` | The course `spawn_robot.launch.py` starts one per robot without a namespace. Our system does not use these nodes (see ARCHITECTURE.md §2). |
| Topics are listed, but no data arrives (`ros2 topic hz /tb3_0/odom` prints nothing, robots ignore `cmd_vel`, markers in view are not detected) | Old Fast-DDS shared-memory files from earlier runs. Stop everything, then `rm -f /dev/shm/fastrtps_*`, `ros2 daemon stop`, `ros2 daemon start`, and launch again. `ros2 topic hz /tb3_0/odom` should show about 30 Hz. To prevent it, use Fast-DDS over UDP (Installation step 5). |
| `spawn_entity`: `Service /spawn_entity unavailable` or the robots never appear | Processes from a previous run are still alive (the launch was killed instead of stopped with Ctrl+C). Check with `ps aux \| grep -E "spawn_entity\|gzserver\|aruco_node"`, kill them, and start again. |
| `colcon build`: `The source ... does not match the source ... used to generate cache` | An older package with the same name was built in this workspace. Delete `~/ros2_ws/build/<package>` and `~/ros2_ws/install/<package>` and build again. |
| `test_flake8` fails with `'Application' object has no attribute 'parse_preliminary_options_and_args'` | A pip-installed flake8 (newer than 3.7.9) overrides the one ROS Foxy needs. Run the tests with `PYTHONNOUSERSITE=1 python3 -m pytest test/`, or remove the pip flake8 (`pip3 uninstall flake8`). |
| `teleop_twist_keyboard` does not drive with `w` | In this program `w` raises the speed; the movement keys are `i` (forward), `,` (back), `j`/`l` (turn) and `k` (stop). Start it with `--ros-args -r cmd_vel:=/tb3_0/cmd_vel`. |

---

## Use of AI tools

Which AI tools were used, for what, and how the output was checked. Completed
before submission, in line with HVL's guidelines on the use of AI.

---

## References

- [DAT160 Competition description](https://frdedynamics.github.io/hvl_robotics_website/courses/dat160/competition)
- [Course repository `ros2_students_25`](https://github.com/frdedynamics/ros2_students_25)
- P. Corke, *Robotics, Vision and Control*, chapters 4–6.
