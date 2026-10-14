"""Virtual firmware: the robot's wheel drive chain, run against motor models.

Takes ``cmd_vel`` as the real ESP32 would (speed limit, ramp, per-wheel PID on
encoder-measured RPM, PWM duty) and drives two ``MotorModel`` wheels. The
resulting wheel speeds are sent to Gazebo's diff-drive plugin as a body twist
on ``wheel_twist``, so Gazebo still handles ground contact while the wheel
dynamics come from the motor model.

Simulation runs in fixed sub-steps on simulation time: motor physics every
``physics_dt``, one firmware main-loop pass every ``loop_period``.

Parameters use the firmware's names (``motor.driver.pid.kp``, ...), and the
same ones can be changed while running: PID gains and period, ramp
acceleration. Read-only status parameters ``motor.{left,right}.{rpm.target,
rpm.now,pwm.now}`` are refreshed every 0.5 s, like the firmware's, so tools
written for the robot (``firmware_monitor``) work unchanged in simulation.
Model-only parameters live under ``motor_model.*``.

Debug topics (sensor_msgs/JointState, joints ``[left, right]``, rad/s):
  firmware/wheel_target    ramped target speed the PID tracks
  firmware/wheel_measured  speed the firmware measures from encoder ticks
  firmware/wheel_true      true model speed; ``effort`` = PWM duty
"""
import math

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from bld120_sim.firmware_model import (MotorModel, RPM_PER_RAD_S, SampledPI, SpeedRamp,
                                       WheelController)
from bld120_sim.ros_util import run_node

STATUS_PARAMS = tuple(f'motor.{side}.{name}' for side in ('left', 'right')
                      for name in ('rpm.target', 'rpm.now', 'pwm.now'))
LIVE_PARAMS = ('motor.driver.pid.kp', 'motor.driver.pid.ki', 'motor.driver.pid.kd',
               'motor.driver.pid.period', 'base.wheel.accel.max')


class VirtualFirmware(Node):

    def __init__(self):
        super().__init__('virtual_firmware')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        # Same names as the firmware's micro-ROS parameters on the robot's /pet node.
        self.wheel_dia = p('base.wheel.diameter', 0.043)
        self.wheel_track = p('base.wheel.track', 0.105043)
        self.ticks_per_rev = 4 * p('motor.encoder.ppr', 1035)    # quadrature
        max_rpm = p('motor.rpm.max', 180.0)
        accel_max = p('base.wheel.accel.max', 2.0)
        kp, ki, kd, pid_period = (p(f'motor.driver.pid.{k}', d) for k, d in
                                  (('kp', 0.001), ('ki', 0.003), ('kd', 0.0), ('period', 0.03)))
        # Simulation only.
        no_load_rpm = p('motor_model.no_load_rpm', 200.0)
        tau = p('motor_model.tau', 0.05)
        dead_zone = p('motor_model.dead_zone', 0.10)
        self.physics_dt = p('physics_dt', 0.001)
        self.loop_period = p('loop_period', 0.002)
        tick_period = p('tick_period', 0.005)
        self.status_period = p('status_period', 0.5)    # firmware: UROS_PARAMS_UPDATE_PERIOD_US
        self.joint_names = [p('joints.left', 'wheel_left_joint'),
                            p('joints.right', 'wheel_right_joint')]
        for name in STATUS_PARAMS:
            self.declare_parameter(name, 0.0)

        self.ramp = SpeedRamp(self.wheel_dia, self.wheel_track, max_rpm, accel_max)
        self.controllers = [
            WheelController(SampledPI(kp, ki, pid_period, kd), self.ticks_per_rev, max_rpm,
                            pid_period)
            for _ in range(2)]
        self.motors = [MotorModel(no_load_rpm, tau, dead_zone) for _ in range(2)]
        self.add_on_set_parameters_callback(self.on_set_parameters)
        self.updating_status = False

        self.prev_time = None
        self.loop_clock = 0.0
        self.status_clock = 0.0
        self.twist_pub = self.create_publisher(Twist, 'wheel_twist', 10)
        self.debug_pubs = {name: self.create_publisher(JointState, f'firmware/{name}', 10)
                           for name in ('wheel_target', 'wheel_measured', 'wheel_true')}
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(tick_period, self.on_tick)
        self.get_logger().info(
            f'virtual firmware: PID kp={kp:g} ki={ki:g} kd={kd:g} @ {pid_period * 1e3:g} ms, '
            f'ramp {accel_max:g} m/s^2, motor {no_load_rpm:g} rpm / '
            f'tau {tau * 1e3:g} ms / dead zone {dead_zone:g}')

    def on_set_parameters(self, params):
        """Live changes, like ``ros2 param set /pet ...`` on the robot."""
        for param in params:
            name, value = param.name, param.value
            if name in STATUS_PARAMS:
                if not self.updating_status:
                    return SetParametersResult(successful=False, reason=f'{name} is read-only')
            elif name in LIVE_PARAMS:
                if name == 'base.wheel.accel.max':
                    self.ramp.accel_max = value
                for c in self.controllers:
                    if name == 'motor.driver.pid.period':
                        c.pid.reference_period = value
                        c.pid_period = value
                    elif name.startswith('motor.driver.pid.'):
                        setattr(c.pid, name.rsplit('.', 1)[1], value)   # kp, ki, kd
                self.get_logger().info(f'{name} = {value:g}')
            elif self.prev_time is not None:
                return SetParametersResult(successful=False,
                                           reason=f'{name} cannot change while running')
        return SetParametersResult(successful=True)

    def publish_status(self):
        """Refresh the read-only status parameters, as the firmware does every 0.5 s."""
        values = []
        for side, c in zip(('left', 'right'), self.controllers):
            values += [Parameter(f'motor.{side}.rpm.target', value=float(c.target_rpm)),
                       Parameter(f'motor.{side}.rpm.now', value=float(c.measured_rpm)),
                       Parameter(f'motor.{side}.pwm.now', value=float(c.duty))]
        self.updating_status = True
        try:
            self.set_parameters(values)
        finally:
            self.updating_status = False

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
            self.status_clock += dt
            for controller, motor in zip(self.controllers, self.motors):
                motor.step(controller.duty, dt)
        self.publish(now.to_msg())
        if self.status_clock >= self.status_period:
            self.status_clock = 0.0
            self.publish_status()

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
