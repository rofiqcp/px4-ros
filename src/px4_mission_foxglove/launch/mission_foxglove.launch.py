"""One USB owner and read-only PX4 mission/Foxglove visualization stack."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, PackageNotFoundError

def generate_launch_description():
    args=[
        DeclareLaunchArgument('start_usb_bridge',default_value='false',
            description='Enable ONLY if no other MAVLink program owns the Cube serial port'),
        DeclareLaunchArgument('start_foxglove',default_value='true'),
        DeclareLaunchArgument('start_calibration_web',default_value='true'),
        DeclareLaunchArgument('web_calibration_enabled',default_value='false'),
        DeclareLaunchArgument('px4_calibration_enabled',default_value='false'),
        DeclareLaunchArgument('start_lidar',default_value='false',
            description='LiDAR obstacle adapter expects Livox topic /livox/lidar'),
    ]
    nodes=[
        Node(package='pixhawk_bridge',executable='pixhawk_node',namespace='pixhawk',
            name='pixhawk_bridge',condition=IfCondition(LaunchConfiguration('start_usb_bridge')),
            parameters=[{
                'device':'/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00',
                'baud':115200,'enable_motion_commands':False,
                'enable_avoidance_advisory_tx':False,
                'avoidance_firmware_confirmed':False,'allow_parameter_writes':False,
                'allow_sensor_calibration':ParameterValue(LaunchConfiguration('px4_calibration_enabled'),value_type=bool),
                'allow_mode_changes':False,'allow_mission_writes':False,
                'publish_raw_mavlink':False,
            }],output='screen'),
        Node(package='px4_mission_foxglove',executable='mission_observer',
            name='mission_observer',output='screen',
            parameters=[__import__('os').path.join(
                get_package_prefix('px4_mission_foxglove'),'share',
                'px4_mission_foxglove','config','mission_observer.yaml')]),
        Node(package='px4_calibration_web',executable='calibration_web',
            name='px4_calibration_web',output='screen',
            condition=IfCondition(LaunchConfiguration('start_calibration_web')),
            parameters=[{'address':'127.0.0.1','port':8093,
                'enable_sensor_calibration':ParameterValue(LaunchConfiguration('web_calibration_enabled'),value_type=bool)}]),
        Node(package='px4_vesc_interface',executable='vesc_preview',
            name='vesc_preview',output='screen',
            parameters=[__import__('os').path.join(
                get_package_prefix('px4_vesc_interface'),'share',
                'px4_vesc_interface','config','vesc_interface.yaml')]),
        Node(package='pixhawk_avoidance_adapter',executable='obstacle_advisory_node',
            name='obstacle_advisory_node',
            condition=IfCondition(LaunchConfiguration('start_lidar')),
            output='screen'),
        Node(package='stock_px4_supervisor',executable='stock_px4_supervisor',
            name='stock_px4_supervisor',output='screen'),
    ]
    try:get_package_prefix('foxglove_bridge')
    except PackageNotFoundError:
        nodes.append(LogInfo(msg='foxglove_bridge NOT INSTALLED; install official ROS 2 bridge to use live Foxglove WebSocket'))
    else:
        nodes.append(Node(package='foxglove_bridge',executable='foxglove_bridge',
            name='px4_foxglove_bridge',
            condition=IfCondition(LaunchConfiguration('start_foxglove')),
            parameters=[{
                'address':'127.0.0.1','port':8765,
                'num_threads':2,
                'topic_whitelist':['^/pixhawk/.*$','^/px4_mission/.*$',
                    '^/vesc_ackermann/.*$','^/livox/.*$','^/lio/.*$','^/p4/.*$',
                    '^/tf(_static)?$','^/diagnostics$','^/parameter_events$'],
                'service_whitelist':['(?!)'],
                'client_topic_whitelist':['(?!)'],
                'param_whitelist':[
                    '^/mission_observer/(mission_refresh_s|status_timeout_s|attitude_timeout_s|advisory_timeout_s|cruise_speed_mps|max_yaw_rate_radps|heading_kp|mission_arrival_tolerance_m)$',
                    '^/vesc_preview/(wheelbase_m|track_width_m|wheel_radius_m|motor_pole_pairs|transmission_ratio|max_speed_mps|max_steer_rad|max_erpm)$',
                ],
                'capabilities':['connectionGraph','assets','parameters','parametersSubscribe'],
                'send_buffer_limit':20000000,'min_qos_depth':1,'max_qos_depth':10,
            }],output='screen'))
    return LaunchDescription(args+nodes)
