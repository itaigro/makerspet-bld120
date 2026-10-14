"""Run one recorded experiment on the real robot (or against the simulation).

Start the robot as usual first (micro-ROS agent + upstream's
``kaiaai_bringup physical.launch.py``), then in another terminal:

  ros2 launch bld120_bringup experiment.launch.py experiment:=steps
  ros2 launch bld120_bringup experiment.launch.py experiment:=square direction:=cw
  ros2 launch bld120_bringup experiment.launch.py experiment:=staircase   # wheels up!
  ros2 launch bld120_bringup experiment.launch.py experiment:=line dry_run:=true

Each run gets its own folder ``output_dir/<experiment>_<YYYYmmdd_HHMMSS>/``:
  run.yaml            launch arguments and start time
  bag/                rosbag: telemetry, joint_states, odom, cmd_vel, battery_state
  firmware.csv/.json  firmware status parameters (PWM duty...) and configuration
  steps.csv           step_test recording (steps, staircase)
  odom.csv            odometry poses (square, line)

Everything shuts down when the experiment ends; Ctrl-C stops the robot.
Against the simulation: ``use_sim_time:=true firmware_node:=/virtual_firmware``.
See docs/PROTOCOL.md for the full measurement procedure.
"""
from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

EXPERIMENTS = ('steps', 'staircase', 'square', 'line')
BAG_TOPICS = ('/telemetry', '/joint_states', '/odom', '/cmd_vel', '/battery_state')


def make_run(context):
    def arg(name):
        return context.perform_substitution(LaunchConfiguration(name))

    experiment = arg('experiment')
    sim = arg('use_sim_time') == 'true'
    dry_run = arg('dry_run') == 'true'
    started = datetime.now()
    run_dir = os.path.join(os.path.expanduser(arg('output_dir')),
                           f'{experiment}_{started:%Y%m%d_%H%M%S}')
    common = {'use_sim_time': sim}

    if experiment in ('steps', 'staircase'):
        test = Node(package='bld120_sim', executable='step_test', output='screen',
                    parameters=[{**common, 'profile': experiment, 'dry_run': dry_run,
                                 'output': os.path.join(run_dir, 'steps.csv')}])
    else:
        test = Node(package='bld120_sim', executable='square_driver', output='screen',
                    parameters=[{**common, 'shape': experiment, 'dry_run': dry_run,
                                 'side': float(arg('side')), 'direction': arg('direction'),
                                 'linear_speed': float(arg('linear_speed')),
                                 'angular_speed': float(arg('angular_speed'))}])

    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=test,
        on_exit=[TimerAction(period=1.5, actions=[EmitEvent(event=Shutdown(
            reason=f'{experiment} finished'))])]))
    if dry_run:
        return [test, shutdown]

    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, 'run.yaml'), 'w') as f:
        f.write(f'started: {started.isoformat(timespec="seconds")}\n')
        for name in ('experiment', 'use_sim_time', 'firmware_node', 'side', 'direction',
                     'linear_speed', 'angular_speed'):
            f.write(f'{name}: {arg(name)}\n')
        f.write('notes: ""   # floor, battery, anything unusual\n')

    actions = [LogInfo(msg=f'run folder: {run_dir}')]
    if arg('record_bag') == 'true':
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(run_dir, 'bag'),
                 *(['--use-sim-time'] if sim else []), *BAG_TOPICS],
            output='log'))
    actions.append(Node(
        package='bld120_sim', executable='firmware_monitor', output='screen',
        parameters=[{**common, 'firmware_node': arg('firmware_node'),
                     'output': os.path.join(run_dir, 'firmware.csv')}]))
    if experiment in ('square', 'line'):
        topics = ['odom', 'ground_truth/odom'] if sim else ['odom']
        actions.append(Node(
            package='bld120_sim', executable='pose_recorder', output='screen',
            parameters=[{**common, 'topics': topics,
                         'output': os.path.join(run_dir, 'odom.csv')}]))
    # Let the recorders subscribe and the firmware configuration be read first.
    actions += [TimerAction(period=float(arg('start_delay')), actions=[test]), shutdown]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('experiment', choices=list(EXPERIMENTS)),
        DeclareLaunchArgument('output_dir', default_value='~/bld120_runs'),
        DeclareLaunchArgument('dry_run', default_value='false', choices=['true', 'false'],
                              description='Print the plan and exit without moving'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('firmware_node', default_value='/pet',
                              description='/pet on the robot, /virtual_firmware in sim'),
        DeclareLaunchArgument('record_bag', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('start_delay', default_value='3.0',
                              description='Seconds before the robot moves'),
        DeclareLaunchArgument('side', default_value='1.0',
                              description='square side / line length [m]'),
        DeclareLaunchArgument('direction', default_value='ccw', choices=['ccw', 'cw']),
        DeclareLaunchArgument('linear_speed', default_value='0.1'),
        DeclareLaunchArgument('angular_speed', default_value='0.5'),
        OpaqueFunction(function=make_run),
    ])
