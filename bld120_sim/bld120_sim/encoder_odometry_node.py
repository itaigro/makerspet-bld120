"""Simulated wheel odometry that behaves like the real BLD-120MM's.

Subscribes to the true wheel joint angles from Gazebo, runs them through the
encoder and firmware odometry models (``odometry_model``) and publishes
``/odom`` plus the ``odom -> base_footprint`` transform, the same outputs
``kaiaai_telemetry`` produces from the real robot's telemetry.

Calibration error parameters describe how wrong the *firmware's* geometry is
relative to the simulated robot, e.g. ``error.wheel_dia_left_pct: 1.0`` means
the firmware believes the left wheel is 1 % larger than it really is.
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from bld120_sim.odometry_model import EncoderModel, FirmwareOdometry


class EncoderOdometryNode(Node):

    def __init__(self):
        super().__init__('encoder_odometry')
        p = self.declare_parameter
        self.left_joint = p('joints.left', 'wheel_left_joint').value
        self.right_joint = p('joints.right', 'wheel_right_joint').value
        ticks_per_rev = p('encoder.ticks_per_rev', 4140).value
        slip_std = p('encoder.slip_std', 0.0).value
        seed = p('encoder.seed', -1).value
        wheel_dia = p('geometry.wheel_dia', 0.043).value
        wheel_track = p('geometry.wheel_track', 0.105043).value
        err_left = p('error.wheel_dia_left_pct', 0.0).value
        err_right = p('error.wheel_dia_right_pct', 0.0).value
        err_track = p('error.wheel_track_pct', 0.0).value
        rate_hz = p('rate_hz', 20.0).value
        self.odom_frame = p('frames.odom', 'odom').value
        self.base_frame = p('frames.base', 'base_footprint').value
        self.publish_tf = p('publish_tf', True).value

        rng = np.random.default_rng(None if seed < 0 else seed)
        self.enc_left = EncoderModel(ticks_per_rev, slip_std, rng)
        self.enc_right = EncoderModel(ticks_per_rev, slip_std, rng)
        self.odom = FirmwareOdometry(
            wheel_radius_left=0.5 * wheel_dia * (1.0 + err_left / 100.0),
            wheel_radius_right=0.5 * wheel_dia * (1.0 + err_right / 100.0),
            wheel_track=wheel_track * (1.0 + err_track / 100.0),
            ticks_per_rev=ticks_per_rev)

        self.true_angles = None
        self.prev_ticks = None
        self.prev_time = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_timer(1.0 / rate_hz, self.on_timer)
        self.get_logger().info(
            f'encoder odometry: {ticks_per_rev} ticks/rev, {rate_hz:g} Hz, '
            f'calibration error L/R/track = {err_left:+g}/{err_right:+g}/{err_track:+g} %, '
            f'slip std {slip_std:g}')

    def on_joint_states(self, msg: JointState):
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            return
        self.true_angles = (left, right)

    def on_timer(self):
        if self.true_angles is None:
            return
        now = self.get_clock().now()
        ticks = (self.enc_left.update(self.true_angles[0]),
                 self.enc_right.update(self.true_angles[1]))
        if self.prev_ticks is None:
            self.prev_ticks, self.prev_time = ticks, now
            return
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return  # sim clock paused
        self.odom.step(ticks[0] - self.prev_ticks[0], ticks[1] - self.prev_ticks[1], dt)
        self.prev_ticks, self.prev_time = ticks, now
        self.publish(now.to_msg())

    def publish(self, stamp):
        x, y, yaw = self.odom.pose
        qz, qw = math.sin(0.5 * yaw), math.cos(0.5 * yaw)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(self.odom.vel_x)
        msg.twist.twist.angular.z = float(self.odom.vel_yaw)
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
