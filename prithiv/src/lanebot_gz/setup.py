from setuptools import setup

package_name = 'lanebot_gz'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prv',
    maintainer_email='prv@todo.todo',
    description='Gazebo version of the lanebot track',
    license='MIT',
    entry_points={
        'console_scripts': [
            'make_world = lanebot_gz.make_world:main',
            'lane_node = lanebot_gz.lane_node:main',
            'vehicle_node = lanebot_gz.vehicle_node:main',
            'spawn_obstacle = lanebot_gz.spawn_obstacle:main',
        ],
    },
)
