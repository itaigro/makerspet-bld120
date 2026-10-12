"""Shared entry point for this package's nodes."""
import rclpy
from rclpy.executors import ExternalShutdownException


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
