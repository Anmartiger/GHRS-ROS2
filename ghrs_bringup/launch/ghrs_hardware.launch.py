"""
GHRS Hardware-Only Launch File
Launches only the hardware driver nodes (for testing without perception).

Author : Anmar Arafat Al-Momani
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ns = 'tsr1'
    return LaunchDescription([
        Node(package='ghrs_hardware', executable='motor_controller',
             name='motor_controller', namespace=ns,
             parameters=[{'l_rpwm':24,'l_lpwm':23,'r_rpwm':13,'r_lpwm':12,
                          'invert_lr':True}],
             output='screen'),
        Node(package='ghrs_hardware', executable='servo_controller',
             name='servo_controller', namespace=ns,
             parameters=[{'i2c_bus':1,'i2c_addr':0x40}],
             output='screen'),
        Node(package='ghrs_hardware', executable='imu_node',
             name='imu_node', namespace=ns,
             parameters=[{'i2c_bus':1,'i2c_addr':0x4B}],
             output='screen'),
        Node(package='ghrs_hardware', executable='gps_node',
             name='gps_node', namespace=ns,
             parameters=[{'port':'/dev/ttyAMA0','baud':9600}],
             output='screen'),
        Node(package='ghrs_hardware', executable='camera_node',
             name='camera_node', namespace=ns,
             parameters=[{'webcam_device':'/dev/video0'}],
             output='screen'),
        Node(package='ghrs_hardware', executable='pump_led_node',
             name='pump_led_node', namespace=ns,
             parameters=[{'pump_pin':17,'led_pin':27}],
             output='screen'),
    ])
