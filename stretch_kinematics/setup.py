from setuptools import find_packages, setup

package_name = 'stretch_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lamsey',
    maintainer_email='lamsey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematics_node = stretch_kinematics.kinematics_node:main',
            'test_kinematics = stretch_kinematics.test_kinematics:main',
        ],


    },
)
