from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00'),
        DeclareLaunchArgument('baud', default_value='115200'),
        Node(package='pixhawk_bridge', executable='pixhawk_node', name='pixhawk_bridge', namespace='pixhawk',
             output='screen', parameters=[{
                 'device': LaunchConfiguration('device'),
                 'baud': LaunchConfiguration('baud'),
                 'enable_motion_commands': False,
                'enable_avoidance_advisory_tx': False,
                'avoidance_firmware_confirmed': False,
                 'allow_parameter_writes': False,
                 'allow_mode_changes': False,
                 'allow_mission_writes': False,
                 'publish_raw_mavlink': False,
                 'request_auxiliary_sensor_streams': True,
                 'max_position_accuracy_m': 5.0,
                 'max_plausible_speed_mps': 30.0,
                 'heartbeat_timeout_sec': 3.0,
             }])
    ])