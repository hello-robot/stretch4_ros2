from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'stretch_base_hazard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hello Robot Inc.',
    maintainer_email='support@hello-robot.com',
    description=(
        'Robot-centric base hazard map from lidar point clouds and line sensors.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hazard_map_node = stretch_base_hazard.hazard_map_node:main',
            'hazard_cmd_vel_filter_node = stretch_base_hazard.hazard_cmd_vel_filter_node:main',
            'hazard_gamepad_teleop = stretch_base_hazard.hazard_gamepad_teleop:main',
        ],
    },
)
