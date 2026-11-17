from setuptools import find_packages, setup

package_name = 'lanebot_ros'

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
    maintainer='prv',
    maintainer_email='prv@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_straight = lanebot_ros.drive_straight:main',
            'lane_node = lanebot_ros.lane_node:main',
            'control_node = lanebot_ros.control_node:main',
            'obstacle_node = lanebot_ros.obstacle_node:main',
            'spawn_obstacle = lanebot_ros.spawn_obstacle:main',
            'traffic_node = lanebot_ros.traffic_node:main',
            'signals_node = lanebot_ros.signals_node:main',
            'audio_node = lanebot_ros.audio_node:main',
            'cluster_node = lanebot_ros.cluster_node:main',
        ],
    },
)
