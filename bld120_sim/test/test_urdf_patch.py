import os
import xml.etree.ElementTree as ET

import pytest

from bld120_sim.urdf_patch import ODOM_PUBLISHER_PLUGIN, reroute_ground_truth

URDF = f"""<robot name="r">
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <topic>cmd_vel</topic>
    </plugin>
    <plugin filename="{ODOM_PUBLISHER_PLUGIN}" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <odom_topic>/odom</odom_topic>
      <tf_topic>/tf</tf_topic>
    </plugin>
  </gazebo>
</robot>"""


def plugin(xml):
    return next(p for p in ET.fromstring(xml).iter('plugin')
                if p.get('filename') == ODOM_PUBLISHER_PLUGIN)


def test_reroutes_topics_and_frame():
    p = plugin(reroute_ground_truth(URDF))
    assert p.find('odom_topic').text == '/ground_truth/odom'
    assert p.find('tf_topic').text == '/ground_truth/tf'
    assert p.find('odom_frame').text == 'ground_truth_origin'


def test_leaves_other_plugins_alone():
    root = ET.fromstring(reroute_ground_truth(URDF))
    dd = next(p for p in root.iter('plugin') if 'diff-drive' in p.get('filename'))
    assert dd.find('topic').text == 'cmd_vel'


def test_missing_plugin_is_an_error():
    with pytest.raises(ValueError):
        reroute_ground_truth('<robot name="r"/>')


def test_upstream_urdf_has_the_plugin():
    """Guards against upstream renaming the plugin (needs the ROS environment)."""
    pytest.importorskip('xacro')
    ament = pytest.importorskip('ament_index_python.packages')
    try:
        share = ament.get_package_share_directory('makerspet_mini')
    except ament.PackageNotFoundError:
        pytest.skip('makerspet_mini not in the environment')
    import xacro
    xml = xacro.process_file(os.path.join(share, 'urdf', 'robot.urdf.xacro')).toxml()
    assert plugin(reroute_ground_truth(xml)).find('odom_topic').text == '/ground_truth/odom'
