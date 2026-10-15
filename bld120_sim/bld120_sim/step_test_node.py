"""Drive a fixed sequence of velocity steps and record the wheel response.

The same profiles (``experiments.PROFILES``) run in simulation and on the real
robot, where ``/joint_states`` velocities come from the firmware's encoder
measurements (20-40 Hz). ``cmd_vel`` is republished at ``rate`` Hz like teleop or
Nav2 do.

Parameters:
  profile       'steps' (on the floor) or 'staircase' (wheels in the air)
  dry_run       print the profile and exit without moving
  max_distance  guard limit in metres from the start; < 0: automatic (planned
                travel x 1.5 + 0.3 m on the floor, none with wheels in the air)

CSV columns (one row per ``/joint_states`` message, speeds in rad/s):
  t, cmd_v, cmd_w, js_left, js_right,
  target_left, target_right, measured_left, measured_right, duty_left, duty_right
The last six come from the virtual firmware's debug topics and are empty when
it is not running (ideal drive, or the real robot).

The node exits when the profile is finished.
"""
import csv
import os

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bld120_sim.experiments import (MotionGuard, ON_FLOOR, PROFILES, profile_at,
                                    profile_distance, profile_duration)
from bld120_sim.ros_util import run_node, send_stop


class StepTest(Node):

    def __init__(self):
        super().__init__('step_test')
        path = os.path.expanduser(self.declare_parameter('output', 'steps.csv').value)
        rate = self.declare_parameter('rate', 10.0).value
        self.start_delay = self.declare_parameter('start_delay', 2.0).value
        name = self.declare_parameter('profile', 'steps').value
        dry_run = self.declare_parameter('dry_run', False).value
        max_distance = self.declare_parameter('max_distance', -1.0).value
        self.left, self.right = 'wheel_left_joint', 'wheel_right_joint'
        if name not in PROFILES:
            raise ValueError(f'unknown profile {name!r}, choose from {sorted(PROFILES)}')
        self.profile = PROFILES[name]
        planned = profile_distance(self.profile)
        if max_distance < 0.0:
            max_distance = 1.5 * planned + 0.3 if ON_FLOOR[name] else None
        self.guard = MotionGuard(max_distance, profile_duration(self.profile) + 10.0)
        self.describe(name, planned, max_distance)
        if dry_run:
            self.get_logger().info('dry run: not moving')
            raise SystemExit

        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t', 'cmd_v', 'cmd_w', 'js_left', 'js_right',
                              'target_left', 'target_right', 'measured_left',
                              'measured_right', 'duty_left', 'duty_right'])
        self.cmd = (0.0, 0.0)
        self.firmware = {'wheel_target': None, 'wheel_measured': None, 'wheel_true': None}
        self.t0 = None

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 50)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        for topic in self.firmware:
            self.create_subscription(JointState, f'firmware/{topic}',
                                     lambda msg, k=topic: self.firmware.__setitem__(k, msg), 50)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'recording to {path}')

    def describe(self, name, planned, max_distance):
        log = self.get_logger()
        log.info(f'profile {name!r}: {profile_duration(self.profile):g} s, '
                 f'{planned:.2f} m of travel, guard '
                 + (f'{max_distance:.2f} m' if max_distance is not None else 'off'))
        t = 0.0
        for duration, v, w in self.profile:
            log.info(f'  t={t:5.1f} s  {duration:4.1f} s  v={v:+.2f} m/s  w={w:+.2f} rad/s')
            t += duration

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.guard.update_pose(p.x, p.y, self.now())

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_timer(self):
        if self.t0 is None:
            self.t0 = self.get_clock().now()
        t = self.elapsed() - self.start_delay
        if t < 0.0:
            return
        if self.guard.t_start is None:
            self.guard.start(self.now())
        reason = self.guard.check(self.now())
        if reason:
            self.get_logger().error(f'stopping: {reason}')
            raise SystemExit
        cmd = profile_at(self.profile, t)
        if cmd is None:
            self.get_logger().info('profile finished')
            raise SystemExit
        v, w = cmd
        self.cmd = (v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.cmd_pub.publish(msg)

    def wheel(self, msg, field):
        if msg is None:
            return ['', '']
        values = getattr(msg, field)
        out = []
        for name in (self.left, self.right):
            i = msg.name.index(name) if name in msg.name else -1
            out.append(f'{values[i]:.5f}' if 0 <= i < len(values) else '')
        return out

    def on_joint_states(self, msg):
        if self.t0 is None:
            return
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        fw = self.firmware
        self.writer.writerow(
            [f'{t:.4f}', f'{self.cmd[0]:.4f}', f'{self.cmd[1]:.4f}']
            + self.wheel(msg, 'velocity')
            + self.wheel(fw['wheel_target'], 'velocity')
            + self.wheel(fw['wheel_measured'], 'velocity')
            + self.wheel(fw['wheel_true'], 'effort'))

    def on_exit(self):
        send_stop(self.cmd_pub)

    def destroy_node(self):
        if hasattr(self, 'file'):
            self.file.close()
        super().destroy_node()


def main(args=None):
    run_node(StepTest, args)
