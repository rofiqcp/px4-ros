"""VESC diagnostic-only launch: never enable UART motor writes automatically."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    path=os.path.join(get_package_share_directory('px4_vesc_interface'),'config','vesc_interface.yaml')
    return LaunchDescription([Node(package='px4_vesc_interface',executable='vesc_preview',
        name='vesc_preview',parameters=[path],output='screen')])
