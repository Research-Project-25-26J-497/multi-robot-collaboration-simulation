from setuptools import setup
import os
from glob import glob

package_name = 'multi_robot_collab'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sithum',
    maintainer_email='sithum@example.com',
    description='Multi-Robot Collaboration Framework for Autonomous Navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'collaboration_node = multi_robot_collab.collaboration_node:main',
            'advanced_collaboration_node = multi_robot_collab.advanced_collaboration_node:main',
            'simple_controller = multi_robot_collab.simple_controller:main',
            'mock_robot_topics = multi_robot_collab.mock_robot_topics:main',
            'map_merger = multi_robot_collab.map_merger_node:main',
            'scan_merger = multi_robot_collab.scan_merger_node:main',
            'unified_slam = multi_robot_collab.unified_slam_node:main',
            'map_fusion = multi_robot_collab.map_fusion_node:main',
        ],
    },
)