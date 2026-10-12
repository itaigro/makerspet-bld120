"""Virtual firmware: the robot's wheel drive chain, run against motor models.

Takes ``cmd_vel`` as the real ESP32 would (speed limit, ramp, per-wheel PID on
encoder-measured RPM, PWM duty) and drives two ``MotorModel`` wheels. The
resulting wheel speeds are sent to Gazebo's diff-drive plugin as a body twist
on ``wheel_twist``, so Gazebo still handles ground contact while the wheel
dynamics come from the motor model.

Simulation runs in fixed sub-steps on simulation time: motor physics every
``physics_dt``, one firmware main-loop pass every ``loop_period``.

Debug topics (sensor_msgs/JointState, joints ``[left, right]``, rad/s):
  firmware/wheel_target    ramped target speed the PID tracks
  firmware/wheel_measured  speed the firmware measures from encoder ticks
  firmware/wheel_true      true model speed; ``effort`` = PWM duty
"""
import math

from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from bld120_sim.firmware_model import (MotorModel, RPM_PER_RAD_S, SampledPI, SpeedRamp,
                                       WheelController)
from bld120_sim.ros_util import run_node


class VirtualFirmware(Node):

    def __init__(self):
        super().__init__('virtual_firmware')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.wheel_dia = p('geometry.wheel_dia', 0.043)
        self.wheel_track = p('geometry.wheel_track', 0.105043)
        ticks_per_rev = p('encoder.ticks_per_rev', 4140)
        max_rpm = p('motor.max_rpm', 180.0)
        pid_period = p('pid.period', 0.03)
        self.physics_dt = p('physics_dt', 0.001)
        self.loop_period = p('loop_period', 0.002)
        tick_period = p('tick_period', 0.005)
        self.joint_names = [p('joints.left', 'wheel_left_joint'),
                            p('joints.right', 'wheel_right_joint')]

        kp, ki, kd = (p(f'pid.{k}', d) for k, d in (('kp', 0.001), ('ki', 0.003), ('kd', 0.0)))
        no_load_rpm = p('motor.no_load_rpm', 200.0)
        tau = p('motor.tau', 0.05)
        dead_zone = p('motor.dead_zone', 0.10)

        self.ramp = SpeedRamp(self.wheel_dia, self.wheel_track, max_rpm, p('accel_max', 2.0))
        self.controllers = [
            WheelController(SampledPI(kp, ki, pid_period, kd), ticks_per_rev, max_rpm,
                            pid_period)
            for _ in range(2)]
        self.motors = [MotorModel(no_load_rpm, tau, dead_zone) for _ in range(2)]
        self.ticks_per_rev = ticks_per_rev

        self.prev_time = None
        self.loop_clock = 0.0
        self.twist_pub = self.create_publisher(Twist, 'wheel_twist', 10)
        self.debug_pubs = {name: self.create_publisher(JointState, f'firmware/{name}', 10)
                           for name in ('wheel_target', 'wheel_measured', 'wheel_true')}
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(tick_period, self.on_tick)
        self.get_logger().info(
            f'virtual firmware: PID kp={kp:g} ki={ki:g} kd={kd:g} @ {pid_period * 1e3:g} ms, '
            f'ramp {self.ramp.accel_max:g} m/s^2, motor {no_load_rpm:g} rpm / '
            f'tau {tau * 1e3:g} ms / dead zone {dead_zone:g}')

    def on_cmd_vel(self, msg: Twist):
        self.ramp.command(msg.linear.x, msg.angular.z,
                          [c.target_rpm for c in self.controllers])

    def on_tick(self):
        now = self.get_clock().now()
        if self.prev_time is None:
            self.prev_time = now
            return
        elapsed = (now - self.prev_time).nanoseconds * 1e-9
        if elapsed <= 0.0:
            return
        self.prev_time = now
        steps = max(1, round(elapsed / self.physics_dt))
        dt = elapsed / steps
        for _ in range(steps):
            self.loop_clock += dt
            if self.loop_clock >= self.loop_period - 1e-9:
                self.firmware_loop(self.loop_clock)
                self.loop_clock = 0.0
            for controller, motor in zip(self.controllers, self.motors):
                motor.step(controller.duty, dt)
        self.publish(now.to_msg())

    def firmware_loop(self, elapsed):
        for controller, target in zip(self.controllers, self.ramp.update(elapsed)):
            controller.set_target_rpm(target)
        for controller, motor in zip(self.controllers, self.motors):
            ticks = math.floor(motor.angle * self.ticks_per_rev / (2.0 * math.pi))
            controller.update(ticks, elapsed)

    def publish(self, stamp):
        r = 0.5 * self.wheel_dia
        w_left, w_right = (m.omega for m in self.motors)
        twist = Twist()
        twist.linear.x = r * (w_left + w_right) / 2.0
        twist.angular.z = r * (w_right - w_left) / self.wheel_track
        self.twist_pub.publish(twist)

        def joint_state(velocities, efforts=()):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = self.joint_names
            msg.velocity = [float(v) for v in velocities]
            msg.effort = [float(e) for e in efforts]
            return msg

        self.debug_pubs['wheel_target'].publish(joint_state(
            c.target_rpm / RPM_PER_RAD_S for c in self.controllers))
        self.debug_pubs['wheel_measured'].publish(joint_state(
            c.measured_rpm / RPM_PER_RAD_S for c in self.controllers))
        self.debug_pubs['wheel_true'].publish(joint_state(
            (m.omega for m in self.motors), (c.duty for c in self.controllers)))


def main(args=None):
    run_node(VirtualFirmware, args)
