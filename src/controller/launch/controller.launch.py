from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='state_publisher',
            name='gazebo_state_publisher'
        ),
        Node(
            package='controller',
            executable='waypoint_publisher',
            name='waypoint_publisher'
        ),
        Node(package="controller", executable="plotter", name="plotter"),

    ])