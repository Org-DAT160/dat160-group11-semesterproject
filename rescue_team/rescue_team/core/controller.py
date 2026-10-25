"""
Go-to-point controller (assignment A3a; Corke chapter 4, "moving to a point").

Given the robot pose and the next path point, it returns the speeds to send on
cmd_vel. No ROS and no state: robot_handler calls compute() every 0.05 s and
decides itself when a point counts as reached (ARCHITECTURE.md section 6.4).
"""
import math
from typing import Tuple

from rescue_team.core.geometry import distance, heading_to, Point, Pose2D


class GoToPointController:
    """Turn towards the target, then drive to it while correcting the heading."""

    def __init__(self, max_linear_speed: float, max_angular_speed: float,
                 distance_gain: float, heading_gain: float, turn_threshold: float):
        self.max_linear_speed = max_linear_speed      # v_max, m/s
        self.max_angular_speed = max_angular_speed    # limit for omega, rad/s
        self.distance_gain = distance_gain            # k_d, 1/s
        self.heading_gain = heading_gain              # k_h, 1/s
        self.turn_threshold = turn_threshold          # rad

    def compute(self, pose: Pose2D, target: Point) -> Tuple[float, float]:
        """
        Return (v, omega): forward speed in m/s and turn rate in rad/s.

        turn   if |e| > turn_threshold:  v = 0                       omega = k_h * e
        drive  otherwise:                v = min(v_max, k_d * d)     omega = k_h * e
        with d the distance and e the heading error to the target; omega is clamped.
        """
        d = distance((pose.x, pose.y), target)
        e = heading_to(pose, target)

        if abs(e) > self.turn_threshold:
            v = 0.0  # facing too far away: turn in place first (the robot can't move sideways)
        else:
            v = min(self.max_linear_speed, self.distance_gain * d)  # slow down near the point

        omega = self.heading_gain * e
        omega = max(-self.max_angular_speed, min(self.max_angular_speed, omega))
        return v, omega


def _example() -> None:
    """Print one step (python3 -m rescue_team.core.controller)."""
    c = GoToPointController(0.26, 1.5, 1.0, 1.5, 0.5)
    print(c.compute(Pose2D(0.0, 0.0, 0.0), (1.0, math.radians(10))))


if __name__ == '__main__':
    _example()
