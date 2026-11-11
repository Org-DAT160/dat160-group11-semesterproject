# Competition-day checklist

Short version of the README for the day of the competition (INT-05). Details and
troubleshooting: README, "Running" and "Troubleshooting". How the system works:
[SYSTEM_GUIDE.md](SYSTEM_GUIDE.md).

## Before the day

- [ ] `main` is tagged `v-final`, and the organisers have access to the repository.
- [ ] The tag was tested: clean clone, build with the README command, one full run
      on each of the five worlds (`docs/test_runs.md`).
- [ ] Everyone has read `SYSTEM_GUIDE.md`.

## On the competition PC

1. **Code at the tag**
   ```bash
   cd ~/ros2_ws/src
   git clone <repository URL> dat160_group11
   cd dat160_group11 && git checkout v-final
   ```
2. **Course packages and worlds** (README, "Installation"): `multi_robot_challenge_23`,
   `multi_robot_scoring`, `ros2_aruco` in `~/ros2_ws/src`; the scoring worlds copied
   over the challenge worlds; models in `~/.gazebo/models`.
3. **Build**
   ```bash
   source /opt/ros/foxy/setup.bash
   cd ~/ros2_ws
   colcon build --packages-select multi_robot_challenge_23 scoring scoring_interfaces \
     ros2_aruco ros2_aruco_interfaces rescue_interfaces rescue_team
   source install/setup.bash
   ```
4. **Nothing old running**
   ```bash
   pkill -f gzserver; pkill -f gzclient
   ros2 node list          # after a few seconds: nothing
   ```

## Start the run

**We start the world** (variant `wN` chosen by the organisers):
```bash
ros2 launch rescue_team rescue.launch.py variant:=wN     # terminal 1
ros2 run scoring scoring                                 # terminal 2, after both robots stand
```

**The organisers start the world themselves:**
```bash
ros2 launch rescue_team rescue.launch.py start_world:=false
```
Our nodes read the map, the start poses and the world from the running system, so
`variant` is not needed here.

The robots start on their own when the leader logs `Started:`. Do not touch
anything during the run.

## What to watch

| When | Log line (leader) | Expected |
|---|---|---|
| Start-up | `waiting for start-up: ...` every 5 s | Lists what is missing; disappears within about 30 s |
| Start-up | `viewpoints computed: N, M` | N about 20–32, M about 20–28 |
| Start | `Started: tb3_0 at ..., tb3_1 at ...` | The positions of the two spawn points |
| Run | `marker K: ACCEPTED at (x, y)` | One per marker; the scoring terminal's score rises by 100 |
| Run | `meeting near ...` then `state MEET -> ...` | Score +300 within a few seconds after both arrive |
| End | `MISSION FINISHED after ... s: all markers accepted, meeting held=True` | Scoring stops its clock |

`gz stats` should show a real-time factor near 1.0; scoring counts wall time.

## If something goes wrong

| Symptom | Action |
|---|---|
| `waiting for start-up` does not go away | Read what it lists. `/set_marker_position not available`: scoring is not running. `no odometry`: the robots have not spawned. |
| `has no publisher: markers from tb3_N will be missed` | `aruco_node` is not running for that robot; check the world launch. |
| Every report "not accepted" | An old scoring node is running. Stop everything and start again. |
| A robot stands still for a long time | Normal for up to about 30 s (goal deadline); the leader then gives it a new goal. |

## After the run

- Note the world, the score, the time and any errors.
- Stop everything: Ctrl+C in every terminal, then `pkill -f gzserver; pkill -f gzclient`.
