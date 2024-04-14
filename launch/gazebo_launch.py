from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    cwd = os.getcwd()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_project'),
                    'launch',
                    'sonoma.launch.py'
                ])
            ]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('controller'),
                    'launch',
                    'controller.launch.py'
                ])
            ]),
        ),

        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(cwd, 'launch/config', 'config.rviz')]
        )
    ])