"""One UMBmark-style square run in simulation, recorded to CSV.

Starts the simulation on the flat world (headless by default), records
``/odom`` and ``/ground_truth/odom``, drives a square on odometry feedback and
shuts everything down when the square is finished.

Example:
  ros2 launch bld120_sim square_test.launch.py direction:=cw err_left_pct:=1.0 \\
      output:=~/bld120_runs/cw_err1.csv
  ros2 run bld120_sim analyze_square ~/bld120_runs/*.csv --out ~/bld120_runs
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_ARGS = ('headless', 'world', 'slip_std', 'seed',
            'err_left_pct', 'err_right_pct', 'err_track_pct')


def generate_launch_description():
    lc = LaunchConfiguration
    driver = Node(
        package='bld120_sim', executable='square_driver', output='screen',
        parameters=[{'use_sim_time': True,
                     'side': lc('side'),
                     'direction': lc('direction'),
                     'linear_speed': lc('linear_speed'),
                     'angular_speed': lc('angular_speed')}],
    )
    recorder = Node(
        package='bld120_sim', executable='pose_recorder', output='screen',
        parameters=[{'use_sim_time': True, 'output': lc('output'),
                     'topics': ['odom', 'ground_truth/odom']}],
    )
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bld120_sim'), 'launch', 'sim.launch.py')),
        launch_arguments={**{a: lc(a) for a in SIM_ARGS},
                          'x_pose': '0.0', 'y_pose': '0.0', 'yaw': '0.0'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world', default_value='flat.sdf'),
        DeclareLaunchArgument('slip_std', default_value='0.0'),
        DeclareLaunchArgument('seed', default_value='-1'),
        DeclareLaunchArgument('err_left_pct', default_value='0.0'),
        DeclareLaunchArgument('err_right_pct', default_value='0.0'),
        DeclareLaunchArgument('err_track_pct', default_value='0.0'),
        DeclareLaunchArgument('side', default_value='1.0', description='Square side [m]'),
        DeclareLaunchArgument('direction', default_value='ccw', choices=['ccw', 'cw']),
        DeclareLaunchArgument('linear_speed', default_value='0.1'),
        DeclareLaunchArgument('angular_speed', default_value='0.5'),
        DeclareLaunchArgument('output', default_value='square.csv',
                              description='CSV file for the recorded poses'),
        sim,
        recorder,
        # Give Gazebo time to spawn the robot before driving.
        TimerAction(period=8.0, actions=[driver]),
        RegisterEventHandler(OnProcessExit(
            target_action=driver,
            on_exit=[TimerAction(period=1.0, actions=[EmitEvent(event=Shutdown(
                reason='square finished'))])])),
    ])
