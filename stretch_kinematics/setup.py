from setuptools import find_packages, setup

package_name = 'stretch_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'hello-robot-stretch4-kinematics',
    ],
    url='https://github.com/hello-robot/stretch4_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='Kinematics and velocity control nodes for Stretch 4',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_limiter = stretch_kinematics.nodes.velocity_limiter:main',
            'task_space_controller = stretch_kinematics.nodes.task_space_controller:main',
        ],
    },
)
