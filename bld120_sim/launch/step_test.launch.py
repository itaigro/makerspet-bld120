"""One step-response run in simulation, recorded to CSV.

Starts the simulation on the flat world (headless by default), plays the
``step_test`` velocity profile and shuts everything down when it is finished.

Example:
  ros2 launch bld120_sim step_test.launch.py motor_model:=false output:=~/runs/ideal.csv
  ros2 run bld120_sim analyze_steps ~/runs/ideal.csv ~/runs/motor.csv --out ~/runs
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


def generate_launch_description():
    lc = LaunchConfiguration
    test = Node(
        package='bld120_sim', executable='step_test', output='screen',
        parameters=[{'use_sim_time': True, 'output': lc('output')}],
    )
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('bld120_sim'), 'launch', 'sim.launch.py')),
        launch_arguments={'headless': lc('headless'), 'world': 'flat.sdf',
                          'motor_model': lc('motor_model'),
                          'x_pose': '0.0', 'y_pose': '0.0', 'yaw': '0.0'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('motor_model', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('output', default_value='steps.csv',
                              description='CSV file for the recorded response'),
        sim,
        TimerAction(period=8.0, actions=[test]),
        RegisterEventHandler(OnProcessExit(
            target_action=test,
            on_exit=[TimerAction(period=1.0, actions=[EmitEvent(event=Shutdown(
                reason='step test finished'))])])),
    ])
