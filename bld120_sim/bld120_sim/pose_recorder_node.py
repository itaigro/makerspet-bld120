"""Record planar poses from one or more Odometry topics to a CSV file.

Columns: ``t, topic, x, y, yaw`` with ``t`` in seconds (simulation time when
``use_sim_time`` is set). Rows are flushed as they arrive, so the file is
complete even if the process is killed.
"""
import csv
import math
import os

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class PoseRecorder(Node):

    def __init__(self):
        super().__init__('pose_recorder')
        topics = self.declare_parameter('topics', ['odom', 'ground_truth/odom']).value
        path = os.path.expanduser(self.declare_parameter('output', 'poses.csv').value)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['t', 'topic', 'x', 'y', 'yaw'])
        for topic in topics:
            self.create_subscription(
                Odometry, topic, lambda msg, t=topic: self.on_odom(t, msg), 50)
        self.get_logger().info(f'recording {topics} -> {path}')

    def on_odom(self, topic, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.writer.writerow([f'{t:.3f}', topic.lstrip('/'),
                              f'{p.position.x:.6f}', f'{p.position.y:.6f}', f'{yaw:.6f}'])
        self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
