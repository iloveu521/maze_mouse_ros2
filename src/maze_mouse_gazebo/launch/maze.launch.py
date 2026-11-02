"""Start the maze, Burger, robot TF and optional RViz in one launch."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('maze_mouse_gazebo'))
    tb3_share = Path(get_package_share_directory('turtlebot3_gazebo'))
    # The default maze.world is centered around the world origin; its west
    # entrance is at y ~= -3.44 m. maze_layout.json belongs to maze_6x6.world.
    spawn = {'x': -4.5, 'y': -3.44, 'z': 0.01, 'yaw': 0.0}
    robot_description = (tb3_share / 'urdf' / 'turtlebot3_burger.urdf').read_text()
    model_paths = [str(tb3_share / 'models')]
    gazebo_models = Path('/usr/share/gazebo-11/models')
    if gazebo_models.is_dir():
        model_paths.append(str(gazebo_models))
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])
    model_path = os.pathsep.join(model_paths)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz' / 'maze.rviz')),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('world', default_value=str(share / 'worlds' / 'maze.world')),
        *[DeclareLaunchArgument(key, default_value=str(spawn[key]))
          for key in ('x', 'y', 'z', 'yaw')],
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch' / 'world.launch.py')),
            launch_arguments={
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'world': LaunchConfiguration('world'),
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        ),
        # spawn_entity waits for Gazebo's service; no fragile fixed startup delay.
        Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=[
                '-entity', 'maze_mouse',
                '-file', str(tb3_share / 'models' / 'turtlebot3_burger' / 'model.sdf'),
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
                '-timeout', '90.0',
            ],
        ),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
