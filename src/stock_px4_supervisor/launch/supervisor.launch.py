"""Read-only PX4 stock supervisor; no motor or mission command is ever sent."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config=str(Path(get_package_share_directory('stock_px4_supervisor'))/'config'/'supervisor.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=config),
        Node(package='stock_px4_supervisor',executable='stock_px4_supervisor',
             name='stock_px4_supervisor',output='screen',
             parameters=[LaunchConfiguration('config')]),
    ])
