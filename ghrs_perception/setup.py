from setuptools import setup

package_name = 'ghrs_perception'

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
    description='GHRS perception nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_detection   = ghrs_perception.fire_detection_node:main',
            'turret_tracking  = ghrs_perception.turret_tracking_node:main',
            'obstacle_detect  = ghrs_perception.obstacle_detection_node:main',
            'plant_disease    = ghrs_perception.plant_disease_node:main',
        ],
    },
)
