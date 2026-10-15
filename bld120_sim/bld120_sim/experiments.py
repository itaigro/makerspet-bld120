"""Experiment definitions shared by simulation and the real robot (no ROS here).

* Velocity profiles for ``step_test``.
* ``MotionGuard``: stops an experiment that runs away. The firmware has no
  ``cmd_vel`` timeout, so a robot keeps driving its last command if the laptop,
  the network or the micro-ROS agent fails. The guard cannot help with a lost
  link, but it does catch a robot that drives further or longer than planned,
  or whose odometry stops arriving.
* ``marks_to_pose``: turns the two floor marks of the tape-measure protocol
  (docs/PROTOCOL.md) into the pose of the wheel-axle centre.
"""
import math

# (duration s, linear m/s, angular rad/s)
PROFILES = {
    # Velocity steps on the floor (M2 in simulation, M3 on the robot). Needs
    # about 1.0 m of travel plus a spin in place.
    'steps': (
        (1.0, 0.0, 0.0),
        (3.0, 0.10, 0.0),     # cruise
        (3.0, 0.20, 0.0),     # speed up
        (2.0, 0.0, 0.0),      # stop
        (4.0, 0.02, 0.0),     # creep: below the PWM dead zone?
        (2.0, 0.0, 0.0),
        (3.0, 0.0, 1.0),      # spin in place
        (2.0, 0.0, 0.0),
    ),
    # Wheels in the air: a staircase of speeds, long enough at each level for
    # the firmware's 0.5 s parameter updates to report a settled PWM duty.
    'staircase': tuple(
        seg for v in (0.01, 0.02, 0.03, 0.04, 0.06, 0.08, 0.10, 0.15, 0.20, 0.25, 0.30,
                      0.35, -0.02, -0.05, -0.10, -0.20)
        for seg in ((5.0, v, 0.0), (1.5, 0.0, 0.0))
    ),
}

# Experiments that drive on the floor; the guard's distance limit applies.
ON_FLOOR = {'steps': True, 'staircase': False}


def profile_duration(profile):
    return sum(seg[0] for seg in profile)


def profile_distance(profile):
    """Distance the robot centre travels if it follows the profile exactly."""
    return sum(duration * abs(v) for duration, v, _ in profile)


def profile_at(profile, t):
    """Commanded (v, w) at time ``t`` into the profile, or None once it is over."""
    for duration, v, w in profile:
        if t < duration:
            return v, w
        t -= duration
    return None


class MotionGuard:
    """Decide when an experiment must stop. Feed it poses; ask ``check``.

    max_distance   metres from the start position (None: no limit)
    max_duration   seconds since ``start`` (None: no limit)
    stale_timeout  seconds without a pose before the robot counts as blind
    """

    def __init__(self, max_distance=None, max_duration=None, stale_timeout=0.5):
        self.max_distance = max_distance
        self.max_duration = max_duration
        self.stale_timeout = stale_timeout
        self.t_start = None
        self.origin = None
        self.last_pose_time = None
        self.distance = 0.0

    def start(self, t):
        self.t_start = t

    def update_pose(self, x, y, t):
        if self.origin is None:
            self.origin = (x, y)
        self.distance = math.hypot(x - self.origin[0], y - self.origin[1])
        self.last_pose_time = t

    def check(self, t):
        """Return why the experiment must stop, or None if all is well."""
        if self.t_start is None:
            return None
        if self.last_pose_time is None or t - self.last_pose_time > self.stale_timeout:
            if t - self.t_start > self.stale_timeout:
                return f'no odometry for more than {self.stale_timeout:g} s'
        if self.max_distance is not None and self.distance > self.max_distance:
            return (f'robot is {self.distance:.2f} m from its start, '
                    f'limit {self.max_distance:.2f} m')
        if self.max_duration is not None and t - self.t_start > self.max_duration:
            return f'experiment longer than {self.max_duration:g} s'
        return None


def mark_moves_to_pose(front_move, rear_move, mark_spacing, axle_from_rear):
    """Pose of the axle centre from how far each mark moved from its start mark.

    The protocol's measurement: F1 relative to F0 and R1 relative to R0, as
    (dx, dy) in the start frame. At the start F0 = (mark_spacing, 0), R0 = (0, 0).
    """
    front = (mark_spacing + front_move[0], front_move[1])
    return marks_to_pose(front, rear_move, axle_from_rear)


def marks_to_pose(front, rear, axle_from_rear):
    """Pose (x, y, yaw) of the wheel-axle centre from two floor marks.

    ``front`` and ``rear`` are the (x, y) floor marks under the front and rear
    reference points on the robot's centre line, measured in the start frame
    (x along the start line, y to the left). ``axle_from_rear`` is the distance
    from the rear reference point to the wheel axle, along the centre line.
    """
    dx, dy = front[0] - rear[0], front[1] - rear[1]
    yaw = math.atan2(dy, dx)
    return (rear[0] + axle_from_rear * math.cos(yaw),
            rear[1] + axle_from_rear * math.sin(yaw),
            yaw)
