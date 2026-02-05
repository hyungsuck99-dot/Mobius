from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=['',''], description='YAML parameters file'),
        Node(
            package='tcp_plc_reader',
            executable='tcp_plc_node',
            name='tcp_plc_node',
            output='screen',
            parameters=[params_file],
        ),
    ])

