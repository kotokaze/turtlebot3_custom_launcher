#!/usr/bin/env python3.10

from os import environ
from pathlib import Path
from sys import argv
from typing import Final

from ament_index_python.packages import get_package_share_directory  # type: ignore[import]

from launch import substitutions, LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace, SetRemap


TURTLEBOT3_MODEL: Final[str] = environ['TURTLEBOT3_MODEL']

def generate_launch_description() -> LaunchDescription:
  ld = LaunchDescription()

  num_robots: int = 1

  for arg in argv[1:]:
    if arg.startswith('num_robots') and ':=' in arg:
      num_robots = int(arg.split(':=')[1])
      break


  pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
  pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

  ld.add_action(DeclareLaunchArgument('world_name',
    default_value='turtlebot3_world.world',
    description='Name of world file (will be ignored if world is set)',
    choices=[
      'empty_world.world',
      'turtlebot3_world.world',
      'turtlebot3_dqn_stage1.world',
      'turtlebot3_dqn_stage2.world',
      'turtlebot3_dqn_stage3.world',
      'turtlebot3_dqn_stage4.world',
      'turtlebot3_house.world',
    ],
  ))

  ld.add_action(DeclareLaunchArgument('world',
    default_value=substitutions.PathJoinSubstitution([
      pkg_turtlebot3_gazebo,
      'worlds',
      substitutions.LaunchConfiguration('world_name'),
    ]),
    description='Full path to world file to load',
  ))
  ld.add_action(LogInfo(msg=[
    'Starting Gazebo with ',
    substitutions.LaunchConfiguration('world'),
  ]))

  # Start Gazebo without robots first
  ld.add_action(GroupAction([
    PushRosNamespace('/gazebo'),
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(substitutions.PathJoinSubstitution([
        pkg_gazebo_ros,
        'launch',
        'gazebo.launch.py',
      ])),
      launch_arguments={
        'world': substitutions.LaunchConfiguration('world'),
      }.items(),
    ),
  ]))


  # Launch the robot for the number of passed arguments
  ld.add_action(DeclareLaunchArgument('use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true',
  ))

  ld.add_action(DeclareLaunchArgument('with_drive',
    default_value='false',
    description='Launch with turtlebot3_drive node',
  ))

  model =  Path(pkg_turtlebot3_gazebo).joinpath(
    'models',
    f"turtlebot3_{TURTLEBOT3_MODEL}",
    'model.sdf',
  )

  for idx in range(num_robots):
    name = f'tb3_{idx:02d}'
    namespace = f'/{name:s}' if 1 < num_robots else ''

    robot_state_publisher_cmd = GroupAction([
      PushRosNamespace(namespace),
      SetRemap('/tf', 'tf'),
      SetRemap('/tf_static', 'tf_static'),
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource(substitutions.PathJoinSubstitution([
          pkg_turtlebot3_gazebo,
          'launch',
          'robot_state_publisher.launch.py',
        ])),
      ),
    ])

    spawn_turtlebot3_cmd = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      name='spawn_entity',
      arguments=[
        '-file', str(model),
        '-entity', name,
        '-robot_namespace', namespace,
        '-unpause',
        '-x', f'{(-idx):.1f}',
        '-y', f'{(0):.1f}',
        '-z', '0.01',
      ],
      output='screen',
    )

    drive_turtlebot3_cmd = Node(
      condition=IfCondition(substitutions.LaunchConfiguration('with_drive')),
      package='turtlebot3_gazebo',
      executable='turtlebot3_drive',
      name='turtlebot3_drive',
      namespace=namespace,
      output='screen',
    )

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot3_cmd)
    ld.add_action(drive_turtlebot3_cmd)

  return ld
