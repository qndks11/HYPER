import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ★ launch 폴더 (odometry.py 설치)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # ★ params 폴더 (dual_ekf_navsat.yaml 설치)
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bean1120',
    maintainer_email='bean1120@todo.todo',
    description='EKF sensor fusion (encoder + IMU + GPS) using robot_localization',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)