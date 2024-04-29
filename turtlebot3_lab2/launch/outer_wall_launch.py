from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='turtlebot3_lab2',
            executable='outer_wall',
            name='turtle_control'
        )
    ])