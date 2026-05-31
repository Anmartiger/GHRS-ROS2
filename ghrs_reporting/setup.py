from setuptools import setup
package_name = 'ghrs_reporting'
setup(
    name=package_name, version='1.0.0', packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Anmar Arafat Al-Momani', maintainer_email='anmar.arafat@tsr1.dev',
    description='GHRS report generation node', license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'report_node = ghrs_reporting.report_node:main',
    ]},
)
