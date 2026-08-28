import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'star_nose_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='divuthejo',
    maintainer_email='djdivyajoshi774@gmail.com',
    description='ROS2 integration for the 19-point KYWO tactile sensor + UR5 rig.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = star_nose_ros2.ui_node:main',
            'ur5_node = star_nose_ros2.ur5_node:main',
            'data_logger_node = star_nose_ros2.data_logger_node:main',
            'friction_ui_node = star_nose_ros2.friction_ui_node:main',
            'ur5_friction_node = star_nose_ros2.ur5_friction_node:main',
        ],
    },
)
