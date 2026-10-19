from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    share=get_package_share_directory('pixhawk_avoidance_adapter')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=os.path.join(share,'config','avoidance.yaml')),
        Node(package='pixhawk_avoidance_adapter',executable='obstacle_advisory_node',
             name='obstacle_advisory_node',output='screen',
             parameters=[LaunchConfiguration('config')])
    ])
