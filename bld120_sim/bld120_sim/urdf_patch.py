"""Launch-time edits to the upstream ``makerspet_mini`` URDF.

Upstream's Gazebo model publishes the *true* model pose as ``/odom`` and as the
``odom -> base_footprint`` transform (``gz-sim-odometry-publisher-system``), so
simulated odometry never drifts. Instead of copying the upstream URDF, this
module edits the processed XML in memory: the same plugin keeps running but
publishes to ground-truth topics, leaving ``/odom`` and that transform to the
``encoder_odometry`` node.
"""
import xml.etree.ElementTree as ET

ODOM_PUBLISHER_PLUGIN = 'gz-sim-odometry-publisher-system'


def _set_child_text(parent, tag, text):
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    child.text = text


def reroute_ground_truth(urdf_xml: str,
                         odom_topic: str = '/ground_truth/odom',
                         tf_topic: str = '/ground_truth/tf',
                         frame: str = 'ground_truth_origin') -> str:
    """Point the upstream odometry-publisher plugin at ground-truth topics.

    Raises ``ValueError`` if the plugin is not found, so a future upstream
    change cannot silently bring back drift-free odometry.
    """
    root = ET.fromstring(urdf_xml)
    plugins = [p for p in root.iter('plugin')
               if p.get('filename') == ODOM_PUBLISHER_PLUGIN]
    if len(plugins) != 1:
        raise ValueError(
            f'expected exactly one {ODOM_PUBLISHER_PLUGIN} plugin, found {len(plugins)}')
    plugin = plugins[0]
    _set_child_text(plugin, 'odom_topic', odom_topic)
    _set_child_text(plugin, 'tf_topic', tf_topic)
    _set_child_text(plugin, 'odom_frame', frame)
    return ET.tostring(root, encoding='unicode')


DIFF_DRIVE_PLUGIN = 'gz-sim-diff-drive-system'


def free_diff_drive(urdf_xml: str, limit: float = 1000.0) -> str:
    """Raise the diff-drive plugin's acceleration limits so it follows commands.

    Upstream caps linear acceleration at 1 m/s^2. When the virtual firmware's
    motor model produces the wheel speeds, that cap would add a second, fake
    dynamic on top, so it is lifted (linear and angular).
    """
    root = ET.fromstring(urdf_xml)
    plugins = [p for p in root.iter('plugin') if p.get('filename') == DIFF_DRIVE_PLUGIN]
    if len(plugins) != 1:
        raise ValueError(
            f'expected exactly one {DIFF_DRIVE_PLUGIN} plugin, found {len(plugins)}')
    for tag in ('max_linear_acceleration', 'max_angular_acceleration'):
        _set_child_text(plugins[0], tag, str(limit))
    return ET.tostring(root, encoding='unicode')
