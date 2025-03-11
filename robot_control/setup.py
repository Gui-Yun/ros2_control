from setuptools import setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 robot control package for ESP32-based robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_control.robot_controller:main',
            'teleop_keyboard = robot_control.teleop_keyboard:main',
        ],
    },
) 