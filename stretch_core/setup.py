from setuptools import setup, find_packages

package_name = 'stretch_core'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    url='https://github.com/hello-robot/stretch_ros2',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The stretch_core package',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stretch_driver = stretch_core.stretch_driver:main',
            'usb_cam = stretch_core.usb_cam:main',
            'remote_gamepad = stretch_core.remote_gamepad:main',
            'luxonis_camera_node = stretch_core.luxonis_camera_node:main',
            'camera_info_publisher = stretch_core.camera_info_publisher:main',
        ],
    },
)
