"""Gazebo simulation of the BLD-120MM with firmware-realistic wheel odometry.

Reuses upstream's robot model (``makerspet_mini``) and worlds (``kaiaai_gazebo``)
unchanged on disk. Differences from ``kaiaai_gazebo/world.launch.py``:

* The model's ground-truth odometry plugin is rerouted to ``/ground_truth/odom``
  (see ``bld120_sim.urdf_patch``); ``/odom`` and ``odom -> base_footprint`` come
  from the ``encoder_odometry`` node instead.
* ``headless:=true`` runs the Gazebo server only (for tests and benchmarks).
* ``world`` accepts a file name in ``bld120_sim/worlds`` (e.g. ``flat.sdf``), in
  ``kaiaai_gazebo/worlds`` (e.g. ``living_room.world``), or a full path.

Example:
  ros2 launch bld120_sim sim.launch.py world:=living_room.world
  ros2 launch bld120_sim sim.launch.py headless:=true slip_std:=0.02
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from bld120_sim.gazebo_paths import gazebo_env
from bld120_sim.urdf_patch import reroute_ground_truth

ROBOT_MODEL = 'makerspet_mini'


def resolve_world(world):
    if os.path.isabs(world):
        return world
    for package in ('bld120_sim', 'kaiaai_gazebo'):
        candidate = os.path.join(get_package_share_directory(package), 'worlds', world)
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(f'world {world!r} not found in bld120_sim or kaiaai_gazebo')


def gazebo(world, headless, verbosity):
    """Run ``gz sim`` directly rather than through ros_gz_sim's gz_sim.launch.py.

    That launch file starts Gazebo with ``shell=True``, so a launch shutdown
    signals only ``/bin/sh`` and the Gazebo server keeps running as an orphan
    (only an interactive Ctrl-C, which signals the whole process group, stops
    it). Without the shell, the signal reaches ``gz`` itself. The search paths
    that launch file would set are reproduced by ``gazebo_env()``.
    """
    cmd = ['gz', 'sim', '-r', '-v', verbosity] + (['-s'] if headless else []) + [world]
    return ExecuteProcess(
        cmd=cmd, name='gazebo', output='screen',
        additional_env=gazebo_env(),
        on_exit=Shutdown(reason='Gazebo exited'))


def make_nodes(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    headless = arg('headless').lower() == 'true'
    world = resolve_world(arg('world'))
    share = get_package_share_directory('bld120_sim')

    urdf_path = os.path.join(
        get_package_share_directory(ROBOT_MODEL), 'urdf', 'robot.urdf.xacro')
    robot_description = reroute_ground_truth(xacro.process_file(urdf_path).toxml())

    odom_params = os.path.join(share, 'config', 'encoder_odometry.yaml')
    overrides = {
        'use_sim_time': True,
        'encoder.slip_std': float(arg('slip_std')),
        'encoder.seed': int(arg('seed')),
        'error.wheel_dia_left_pct': float(arg('err_left_pct')),
        'error.wheel_dia_right_pct': float(arg('err_right_pct')),
        'error.wheel_track_pct': float(arg('err_track_pct')),
    }

    return [
        gazebo(world, headless, arg('gz_verbosity')),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
            output='screen',
            parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge.yaml'),
                         'use_sim_time': True}],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', ROBOT_MODEL, '-topic', 'robot_description',
                       '-x', arg('x_pose'), '-y', arg('y_pose'), '-z', '0.01',
                       '-Y', arg('yaw'), '-allow_renaming', 'false'],
        ),
        Node(
            package='bld120_sim', executable='encoder_odometry', output='screen',
            parameters=[odom_params, overrides],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='living_room.world',
                              description='World in bld120_sim/worlds, kaiaai_gazebo/worlds, '
                                          'or a full path'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='Run the Gazebo server only, without the GUI'),
        DeclareLaunchArgument('gz_verbosity', default_value='2', choices=['0', '1', '2', '3', '4'],
                              description='Gazebo log level: 2 = errors and warnings, '
                                          '4 = debug (what kaiaai_gazebo uses)'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-0.5'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('slip_std', default_value='0.0',
                              description='Random wheel slip: std of the per-step '
                                          'multiplicative noise'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Slip noise seed; -1 = random'),
        DeclareLaunchArgument('err_left_pct', default_value='0.0',
                              description='Firmware left wheel diameter error, percent'),
        DeclareLaunchArgument('err_right_pct', default_value='0.0',
                              description='Firmware right wheel diameter error, percent'),
        DeclareLaunchArgument('err_track_pct', default_value='0.0',
                              description='Firmware wheel track error, percent'),
        OpaqueFunction(function=make_nodes),
    ])
