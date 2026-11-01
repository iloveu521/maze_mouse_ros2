"""Launch C++20 navigation, optionally with the Gazebo environment."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('maze_mouse_navigation'))
    gazebo = Path(get_package_share_directory('maze_mouse_gazebo'))
    return LaunchDescription([
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('controller', default_value='left_wall_follower',
                             choices=['left_wall_follower', 'maze_navigator']),
        DeclareLaunchArgument('params', default_value=PathJoinSubstitution([
            str(share), 'config', PythonExpression([
                "'left_wall.yaml' if '", LaunchConfiguration('controller'),
                "' == 'left_wall_follower' else 'navigation.yaml'",
            ]),
        ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'maze.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'rviz': LaunchConfiguration('rviz'),
                              'rviz_config': str(share / 'rviz' / 'navigation.rviz')}.items(),
            condition=IfCondition(LaunchConfiguration('simulation')),
        ),
        Node(package='maze_mouse_navigation', executable=LaunchConfiguration('controller'),
             name='maze_navigator', output='screen',
             parameters=[LaunchConfiguration('params')]),
    ])
