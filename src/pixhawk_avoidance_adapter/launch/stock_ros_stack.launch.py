"""Production-safe companion advisory stack. Uses existing LiDAR and MAVLink bridge.
Never launches, arms, flashes or reconfigures the Cube Pixhawk.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    perception = Path(get_package_share_directory('pixhawk_avoidance_adapter')) / 'config' / 'avoidance.yaml'
    supervisor_dir = Path(get_package_share_directory('stock_px4_supervisor'))
    supervisor = supervisor_dir / 'config' / 'supervisor.yaml'
    differential = supervisor_dir / 'config' / 'differential.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('perception_config', default_value=str(perception)),
        DeclareLaunchArgument('supervisor_config', default_value=str(supervisor)),
        DeclareLaunchArgument('differential_config', default_value=str(differential)),
        Node(
            package='pixhawk_avoidance_adapter',
            executable='obstacle_advisory_node',
            name='obstacle_advisory_node',
            output='screen',
            parameters=[LaunchConfiguration('perception_config')],
        ),
        Node(
            package='stock_px4_supervisor',
            executable='stock_px4_supervisor',
            name='stock_px4_supervisor',
            output='screen',
            parameters=[LaunchConfiguration('supervisor_config')],
        ),
        Node(
            package='stock_px4_supervisor',
            executable='differential_advisor',
            name='differential_advisor',
            output='screen',
            parameters=[LaunchConfiguration('differential_config')],
        ),
    ])
