"""Drive a fixed sequence of velocity steps and record the wheel response.

The same profile runs in simulation and (M3) on the real robot, where
``/joint_states`` velocities come from the firmware's encoder measurements.
``cmd_vel`` is republished at ``rate`` Hz like teleop or Nav2 do.

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
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bld120_sim.ros_util import run_node

# (duration s, linear m/s, angular rad/s)
PROFILE = (
    (1.0, 0.0, 0.0),
    (3.0, 0.10, 0.0),     # cruise
    (3.0, 0.20, 0.0),     # speed up
    (2.0, 0.0, 0.0),      # stop
    (4.0, 0.02, 0.0),     # creep: below the PWM dead zone?
    (2.0, 0.0, 0.0),
    (3.0, 0.0, 1.0),      # spin in place
    (2.0, 0.0, 0.0),
)


class StepTest(Node):

    def __init__(self):
        super().__init__('step_test')
        path = os.path.expanduser(self.declare_parameter('output', 'steps.csv').value)
        rate = self.declare_parameter('rate', 10.0).value
        self.start_delay = self.declare_parameter('start_delay', 2.0).value
        self.left, self.right = 'wheel_left_joint', 'wheel_right_joint'

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
        for topic in self.firmware:
            self.create_subscription(JointState, f'firmware/{topic}',
                                     lambda msg, k=topic: self.firmware.__setitem__(k, msg), 50)
        self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f'step profile: {sum(s[0] for s in PROFILE):g} s -> {path}')

    def elapsed(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_timer(self):
        if self.t0 is None:
            self.t0 = self.get_clock().now()
        t = self.elapsed() - self.start_delay
        if t < 0.0:
            return
        for duration, v, w in PROFILE:
            if t < duration:
                break
            t -= duration
        else:
            self.get_logger().info('step profile finished')
            raise SystemExit
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
        self.cmd_pub.publish(Twist())

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    run_node(StepTest, args)
