"""Drive a square using the robot's own odometry (UMBmark-style test).

The robot drives ``side`` metres, turns 90 degrees in place, and repeats four
times, closing the loop on ``/odom`` only, just as it would on the real robot:
leg length, turn angle *and* the heading held on each straight leg all come
from odometry.
If odometry were perfect the robot would end exactly where it started; the true
end-position error (from ground truth in simulation, or a tape measure on the
real robot) measures odometry drift. Running the square in both directions
separates systematic errors (wheel diameter vs. track width, Borenstein &
Feng 1996).

With ``shape:=line`` it drives a single straight leg of ``side`` metres: the
tape-measured distance against the odometry distance calibrates the mean wheel
diameter.

The node exits when the square is finished, so a launch file can shut down
on its exit. A ``MotionGuard`` stops it if the robot leaves the square's area,
takes far longer than planned, or its odometry stops arriving.
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from bld120_sim.experiments import MotionGuard
from bld120_sim.ros_util import run_node, send_stop


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')
        p = self.declare_parameter
        self.side = p('side', 1.0).value
        self.direction = 1.0 if p('direction', 'ccw').value == 'ccw' else -1.0
        self.v_max = p('linear_speed', 0.1).value
        self.w_max = p('angular_speed', 0.5).value
        self.accel = p('linear_accel', 0.2).value          # m/s^2, ramp up and down
        self.alpha = p('angular_accel', 1.0).value         # rad/s^2
        self.k_heading = p('heading_gain', 2.0).value      # (rad/s) per rad of heading error
        self.v_min, self.w_min = 0.02, 0.1                 # creep speeds near the target
        self.start_delay = p('start_delay', 2.0).value     # s, let odometry settle
        shape = p('shape', 'square').value
        dry_run = p('dry_run', False).value
        if shape not in ('square', 'line'):
            raise ValueError(f"shape must be 'square' or 'line', not {shape!r}")
        legs, turns = (4, 4) if shape == 'square' else (1, 0)
        self.n_segments = legs + turns
        reach = self.side * (math.sqrt(2.0) if shape == 'square' else 1.0)
        planned_time = legs * self.side / self.v_max + turns * (math.pi / 2) / self.w_max
        self.guard = MotionGuard(max_distance=reach + 0.3,
                                 max_duration=2.0 * planned_time + 20.0)

        self.pose = None
        self.segment = 0        # even: drive; odd: turn
        self.segment_start = None
        self.start_time = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.02, self.on_timer)
        self.get_logger().info(
            f'{shape}: side {self.side} m, {"CCW" if self.direction > 0 else "CW"}, '
            f'{self.v_max} m/s, {self.w_max} rad/s, about {planned_time:.0f} s; guard '
            f'{self.guard.max_distance:.2f} m / {self.guard.max_duration:.0f} s')
        if dry_run:
            self.get_logger().info('dry run: not moving')
            raise SystemExit

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))
        self.guard.update_pose(p.position.x, p.position.y, self.now())

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        if self.done or self.pose is None:
            return
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if (now - self.start_time).nanoseconds * 1e-9 < self.start_delay:
            return
        if self.guard.t_start is None:
            self.guard.start(self.now())
        reason = self.guard.check(self.now())
        if reason:
            self.get_logger().error(f'stopping: {reason}')
            raise SystemExit
        if self.segment_start is None:
            self.segment_start = self.pose

        cmd = Twist()
        x0, y0, yaw0 = self.segment_start
        x, y, yaw = self.pose
        if self.segment % 2 == 0:
            remaining = self.side - math.hypot(x - x0, y - y0)
            if remaining <= 0.0:
                self.next_segment()
            else:
                cmd.linear.x = self.profile(remaining, self.v_max, self.v_min, self.accel)
                # Hold the leg's starting heading, as measured by odometry: the
                # robot believes it drives straight (UMBmark assumption).
                heading_error = wrap_pi(yaw0 - yaw)
                cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_heading * heading_error))
        else:
            remaining = math.pi / 2 - abs(wrap_pi(yaw - yaw0))
            if remaining <= 0.0:
                self.next_segment()
            else:
                cmd.angular.z = self.direction * self.profile(
                    remaining, self.w_max, self.w_min, self.alpha)
        self.cmd_pub.publish(cmd)

    def on_exit(self):
        send_stop(self.cmd_pub)

    @staticmethod
    def profile(remaining, v_max, v_min, accel):
        """Speed that can still stop within ``remaining`` (v^2 = 2 a d)."""
        return max(v_min, min(v_max, math.sqrt(2.0 * accel * remaining)))

    def next_segment(self):
        self.cmd_pub.publish(Twist())
        self.segment += 1
        self.segment_start = self.pose
        if self.segment == self.n_segments:
            self.done = True
            x, y, yaw = self.pose
            self.get_logger().info(
                f'finished; odometry pose x={x:.4f} m y={y:.4f} m yaw={math.degrees(yaw):.2f} deg')
            raise SystemExit


def main(args=None):
    run_node(SquareDriver, args)
