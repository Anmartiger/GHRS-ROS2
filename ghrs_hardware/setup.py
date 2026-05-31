from setuptools import setup

package_name = 'ghrs_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anmar Arafat Al-Momani',
    maintainer_email='anmar.arafat@tsr1.dev',
    description='GHRS hardware driver nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller  = ghrs_hardware.motor_controller_node:main',
            'servo_controller  = ghrs_hardware.servo_controller_node:main',
            'imu_node          = ghrs_hardware.imu_node:main',
            'gps_node          = ghrs_hardware.gps_node:main',
            'camera_node       = ghrs_hardware.camera_node:main',
            'pump_led_node     = ghrs_hardware.pump_led_node:main',
        ],
    },
)
