import os
import re

import pytest

ament = pytest.importorskip('ament_index_python.packages')


def upstream_urdf():
    try:
        share = ament.get_package_share_directory('makerspet_mini')
    except ament.PackageNotFoundError:
        pytest.skip('makerspet_mini not in the environment')
    xacro = pytest.importorskip('xacro')
    return xacro.process_file(os.path.join(share, 'urdf', 'robot.urdf.xacro')).toxml()


def test_every_robot_mesh_resolves_on_the_gazebo_resource_path():
    """Gazebo maps package://pkg/... to model://pkg/... and searches the resource path."""
    from bld120_sim.gazebo_paths import gazebo_env

    uris = re.findall(r'filename="(?:package|model)://([^"]+)"', upstream_urdf())
    assert uris, 'expected the upstream model to reference at least one mesh'
    roots = gazebo_env({})['GZ_SIM_RESOURCE_PATH'].split(os.pathsep)
    for uri in uris:
        assert any(os.path.isfile(os.path.join(r, uri)) for r in roots), \
            f'{uri} not found under {roots}'


def test_existing_environment_is_kept():
    from bld120_sim.gazebo_paths import gazebo_env

    env = gazebo_env({'GZ_SIM_RESOURCE_PATH': '/mine', 'LD_LIBRARY_PATH': '/lib/x'})
    assert env['GZ_SIM_RESOURCE_PATH'].split(os.pathsep)[0] == '/mine'
    assert '/lib/x' in env['GZ_SIM_SYSTEM_PLUGIN_PATH'].split(os.pathsep)
