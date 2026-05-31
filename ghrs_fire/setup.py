from setuptools import setup
package_name = 'ghrs_fire'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Anmar Arafat Al-Momani', maintainer_email='anmar.arafat@ghrs.dev',
    description='GHRS pesticide spraying coordinator node',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'pesticide_spray = ghrs_fire.fire_suppression_node:main',
    ]},
)
