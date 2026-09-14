import glob
import os

from setuptools import find_packages, setup

package_name = 'hyper_rtk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob.glob('config/*.yaml')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='qndks11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'push_uart2_config = hyper_rtk.push_uart2_config:main',
        ],
    },
)
