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

The node exits when the square is finished, so a launch file can shut down
on its exit.
"""
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from bld120_sim.ros_util import run_node


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

        self.pose = None
        self.segment = 0        # 0, 2, 4, 6: drive; 1, 3, 5, 7: turn
        self.segment_start = None
        self.start_time = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.create_timer(0.02, self.on_timer)
        self.get_logger().info(
            f'square: side {self.side} m, {"CCW" if self.direction > 0 else "CW"}, '
            f'{self.v_max} m/s, {self.w_max} rad/s')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_timer(self):
        if self.done or self.pose is None:
            return
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if (now - self.start_time).nanoseconds * 1e-9 < self.start_delay:
            return
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
        self.cmd_pub.publish(Twist())

    @staticmethod
    def profile(remaining, v_max, v_min, accel):
        """Speed that can still stop within ``remaining`` (v^2 = 2 a d)."""
        return max(v_min, min(v_max, math.sqrt(2.0 * accel * remaining)))

    def next_segment(self):
        self.cmd_pub.publish(Twist())
        self.segment += 1
        self.segment_start = self.pose
        if self.segment == 8:
            self.done = True
            self.get_logger().info(f'square finished, odometry pose {self.pose}')
            raise SystemExit


def main(args=None):
    run_node(SquareDriver, args)
