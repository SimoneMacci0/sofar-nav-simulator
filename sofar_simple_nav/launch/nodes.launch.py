from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sofar_simple_nav',
            executable='robot_controller',
            name='controller_node'
        ),
        Node(
            package='sofar_simple_nav',
            executable='nav_sim',
            name='nav_sim_node'
        ),
        Node(
            package='sofar_simple_nav',
            executable='robot_logic',
            name='robot_logic_node',
        )
    ])
