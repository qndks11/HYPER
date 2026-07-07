from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch 폴더 안의 .py 파일들을 install/perception/share/perception/launch 로 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qndks11',
    maintainer_email='qndks11@hanyang.ac.kr',
    description='Perception package',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_subscriber = perception.camera_subscriber:main',
            'lane_detector = perception.lane_detector:main',
            'sign_detector = perception.sign_detector:main',
        ],
    },
)