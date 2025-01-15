import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'idmind_people_detection'

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
        (os.path.join('share', package_name, 'data'), glob('data/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Carlos Neves',
    maintainer_email='cneves@idmind.pt',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'people_detection = idmind_people_detection.people_detection:main'
        ],
    },
)
