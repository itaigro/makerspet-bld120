"""Shared entry point for this package's nodes."""
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException


def send_stop(publisher, repeats=3, interval=0.05):
    """Publish a zero ``Twist`` a few times.

    The robot's firmware has no command timeout: it keeps driving the last
    ``cmd_vel`` it received, so the stop must get through.
    """
    for i in range(repeats):
        publisher.publish(Twist())
        if i + 1 < repeats:
            time.sleep(interval)


def run_node(node_class, args=None):
    """Spin ``node_class()`` until shutdown, without tracebacks on Ctrl-C.

    Launch forwards SIGINT to every node; a second signal can arrive while the
    node is being destroyed, so cleanup tolerates it too. A node may end itself
    by raising ``SystemExit`` from a callback, and may define ``on_exit()`` for
    last actions (e.g. publishing a stop command).
    """
    rclpy.init(args=args)
    node = None
    try:
        node = node_class()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            if node is not None:
                if hasattr(node, 'on_exit'):
                    try:
                        node.on_exit()
                    except Exception:  # context may already be shut down by a signal
                        pass
                node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
