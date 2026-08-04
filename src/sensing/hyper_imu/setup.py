import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hyper_imu'

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
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='qndks11@gmail.com',
    description='WitMotion BLE IMU driver for HYPER, publishing sensor_msgs/Imu over bleak.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'witmotion_ble_node = hyper_imu.witmotion_ble_node:main',
        ],
    },
)
