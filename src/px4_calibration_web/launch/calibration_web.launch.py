"""Read-only-by-default local PX4 sensor and arming readiness web console."""
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    path=os.path.join(get_package_share_directory('px4_calibration_web'),'config','calibration_web.yaml')
    return LaunchDescription([Node(package='px4_calibration_web',
        executable='calibration_web',name='px4_calibration_web',output='screen',parameters=[path])])
