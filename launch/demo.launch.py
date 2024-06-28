from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qt6_ros2_template',
            executable='qt6_ros2_template_node',
            name='qt6_ros2_template_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
