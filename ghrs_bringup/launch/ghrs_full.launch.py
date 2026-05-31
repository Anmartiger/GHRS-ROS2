"""
GHRS Full System Launch File
==============================
Launches all nodes for a complete GHRS mission.

Usage:
  ros2 launch ghrs_bringup ghrs_full.launch.py
  ros2 launch ghrs_bringup ghrs_full.launch.py ip_cam_rtsp:=rtsp://192.168.1.10/...

Author : Anmar Arafat Al-Momani
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    # ── Declare launch arguments ────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('namespace',        default_value='tsr1'),
        DeclareLaunchArgument('ip_cam_rtsp',       default_value=''),
        DeclareLaunchArgument('ip_cam_user',       default_value=''),
        DeclareLaunchArgument('ip_cam_pass',       default_value=''),
        DeclareLaunchArgument('waypoints_file',    default_value=''),
        DeclareLaunchArgument('model_path',        default_value=''),
        DeclareLaunchArgument('auto_suppress',     default_value='True'),
        DeclareLaunchArgument('report_dir',        default_value='/tmp/ghrs_reports'),
    ]

    ns = LaunchConfiguration('namespace')

    nodes = [
        # ── Hardware layer ─────────────────────────────────────────────────
        Node(
            package='ghrs_hardware', executable='motor_controller',
            name='motor_controller', namespace=ns,
            parameters=[{
                'l_rpwm': 24, 'l_lpwm': 23,
                'r_rpwm': 13, 'r_lpwm': 12,
                'invert_lr': True, 'max_speed': 1.0,
                'wheel_separation': 0.35,
            }],
            output='screen',
        ),
        Node(
            package='ghrs_hardware', executable='servo_controller',
            name='servo_controller', namespace=ns,
            parameters=[{'i2c_bus': 1, 'i2c_addr': 0x40, 'pwm_freq': 50}],
            output='screen',
        ),
        Node(
            package='ghrs_hardware', executable='imu_node',
            name='imu_node', namespace=ns,
            parameters=[{'i2c_bus': 1, 'i2c_addr': 0x4B, 'publish_rate': 50.0}],
            output='screen',
        ),
        Node(
            package='ghrs_hardware', executable='gps_node',
            name='gps_node', namespace=ns,
            parameters=[{'port': '/dev/ttyAMA0', 'baud': 9600}],
            output='screen',
        ),
        Node(
            package='ghrs_hardware', executable='camera_node',
            name='camera_node', namespace=ns,
            parameters=[{
                'webcam_device': '/dev/video0',
                'webcam_width':  640, 'webcam_height': 480,
                'webcam_fps':    30,
                'ip_cam_rtsp':   LaunchConfiguration('ip_cam_rtsp'),
                'ip_cam_user':   LaunchConfiguration('ip_cam_user'),
                'ip_cam_pass':   LaunchConfiguration('ip_cam_pass'),
                'mjpeg_port':    5001,
            }],
            output='screen',
        ),
        Node(
            package='ghrs_hardware', executable='pump_led_node',
            name='pump_led_node', namespace=ns,
            parameters=[{'pump_pin': 17, 'led_pin': 27, 'active_low': True}],
            output='screen',
        ),

        # ── Perception layer ───────────────────────────────────────────────
        Node(
            package='ghrs_perception', executable='fire_detection',
            name='fire_detection', namespace=ns,
            parameters=[{
                'h_low': 0, 'h_high': 35,
                's_low': 100, 's_high': 255,
                'v_low': 100, 'v_high': 255,
                'pixel_ratio_thresh': 0.005,
                'confirm_frames': 3,
            }],
            output='screen',
        ),
        Node(
            package='ghrs_perception', executable='turret_tracking',
            name='turret_tracking', namespace=ns,
            parameters=[{
                'kp_pan': 0.08, 'kd_pan': 0.02,
                'kp_tilt': 0.06, 'kd_tilt': 0.02,
                'fire_deadband': 0.07,
                'suppress_duration': 2.0,
            }],
            output='screen',
        ),
        Node(
            package='ghrs_perception', executable='obstacle_detect',
            name='obstacle_detect', namespace=ns,
            parameters=[{
                'roi_top_frac': 0.55, 'edge_thresh': 0.08,
            }],
            output='screen',
        ),
        Node(
            package='ghrs_perception', executable='plant_disease',
            name='plant_disease', namespace=ns,
            parameters=[{
                'model_path':     LaunchConfiguration('model_path'),
                'scan_rate_hz':   1.0,
                'confidence_min': 0.5,
                'save_dir':       '/tmp/ghrs_samples',
            }],
            output='screen',
        ),

        # ── Fire suppression ───────────────────────────────────────────────
        Node(
            package='ghrs_fire', executable='fire_suppression',
            name='fire_suppression', namespace=ns,
            parameters=[{
                'auto_suppress':    LaunchConfiguration('auto_suppress'),
                'suppress_timeout': 30.0,
                'pump_pre_delay':   0.5,
                'led_strobe_hz':    2.0,
            }],
            output='screen',
        ),

        # ── Navigation ────────────────────────────────────────────────────
        Node(
            package='ghrs_navigation', executable='navigation_node',
            name='navigation_node', namespace=ns,
            parameters=[{
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'arrival_radius': 2.0,
                'patrol_speed':   0.3,
                'turn_speed':     0.5,
            }],
            output='screen',
        ),

        # ── Reporting ─────────────────────────────────────────────────────
        Node(
            package='ghrs_reporting', executable='report_node',
            name='report_node', namespace=ns,
            parameters=[{
                'report_dir': LaunchConfiguration('report_dir'),
                'rover_id':   'GHRS',
                'operator':   'Anmar Arafat Al-Momani',
            }],
            output='screen',
        ),
    ]

    return LaunchDescription(args + [LogInfo(msg='Launching GHRS system...')] + nodes)
