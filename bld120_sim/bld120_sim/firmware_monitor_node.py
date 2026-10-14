"""Log the firmware's live motor parameters (target/measured RPM, PWM duty).

The ESP32 firmware exposes its motor state as ROS parameters on its micro-ROS
node (``/pet``), refreshed every 0.5 s. That is too slow for dynamics, but
it is the only place the PWM duty is visible, so a settled speed level gives
one point of the duty-vs-speed curve. In simulation ``virtual_firmware``
exposes the same parameters.

The firmware's parameter server runs in low-memory mode, which handles one
parameter per request, so the values are fetched one at a time.

Writes ``output`` (CSV: t, then one column per status parameter) and
``output`` with a ``.json`` suffix holding the controller configuration read
at start-up (gains, PID period, ramp), so every run records its settings.
"""
import csv
import json
import os

from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

from bld120_sim.ros_util import run_node

STATUS = tuple(f'motor.{side}.{name}' for side in ('left', 'right')
               for name in ('rpm.target', 'rpm.now', 'pwm.now'))
CONFIG = ('motor.driver.pid.kp', 'motor.driver.pid.ki', 'motor.driver.pid.kd',
          'motor.driver.pid.period', 'base.wheel.accel.max', 'motor.rpm.max',
          'base.wheel.diameter', 'base.wheel.track', 'motor.encoder.ppr')


class FirmwareMonitor(Node):

    def __init__(self):
        super().__init__('firmware_monitor')
        self.target = self.declare_parameter('firmware_node', '/pet').value
        path = os.path.expanduser(self.declare_parameter('output', 'firmware.csv').value)
        period = 1.0 / self.declare_parameter('rate', 2.0).value
        self.json_path = os.path.splitext(path)[0] + '.json'

        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t'] + list(STATUS))

        self.client = AsyncParameterClient(self, self.target)
        self.config = None                 # filled once the firmware answers
        self.queue = []                    # names still to fetch in this round
        self.row = {}
        self.pending = None
        self.waiting_logged = False
        self.create_timer(period, self.on_timer)
        self.get_logger().info(f'polling {self.target} every {period:g} s -> {path}')

    def on_timer(self):
        if self.pending is not None:
            return                          # previous round still in flight
        if not self.client.services_are_ready():
            if not self.waiting_logged:
                self.get_logger().warning(f'waiting for {self.target} parameter services')
                self.waiting_logged = True
            return
        self.queue = list(CONFIG if self.config is None else STATUS)
        self.row = {'t': self.get_clock().now().nanoseconds * 1e-9}
        self.fetch_next()

    def fetch_next(self):
        if not self.queue:
            self.pending = None
            self.finish_round()
            return
        name = self.queue.pop(0)
        self.pending = self.client.get_parameters([name])
        self.pending.add_done_callback(lambda future, n=name: self.on_value(n, future))

    def on_value(self, name, future):
        try:
            values = future.result().values
            self.row[name] = parameter_value_to_python(values[0]) if values else None
        except Exception as err:            # one failed read must not stop the log
            self.get_logger().warning(f'{name}: {err}', throttle_duration_sec=5.0)
            self.row[name] = None
        self.fetch_next()

    def finish_round(self):
        if self.config is None:
            self.config = {k: v for k, v in self.row.items() if k != 't'}
            self.config['firmware_node'] = self.target
            with open(self.json_path, 'w') as f:
                json.dump(self.config, f, indent=2)
            self.get_logger().info(f'firmware configuration: {self.config}')
            return
        self.writer.writerow([f'{self.row["t"]:.3f}'] + [
            '' if self.row.get(k) is None else f'{self.row[k]:.5g}' for k in STATUS])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    run_node(FirmwareMonitor, args)
