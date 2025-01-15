import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'harmony_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Neves',
    maintainer_email='cneves@idmind.pt',
    description='Package that will handle harmony missions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'harmony_mission_node = harmony_mission.harmony_mission_node:main',
            'robot_bridge_node = harmony_mission.robot_bridge:main',
            'map_manager_node = harmony_mission.map_manager:main',
        ],
    },
)
