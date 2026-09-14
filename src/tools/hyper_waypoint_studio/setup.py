import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hyper_waypoint_studio'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Hyeongjun',
    maintainer_email='qndks11@gmail.com',
    description='Waypoint studio: view, edit, record and drive HYPER courses in one window.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_studio = hyper_waypoint_studio.main:main',
        ],
    },
)
