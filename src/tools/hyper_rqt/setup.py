import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hyper_rqt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kim Hyeongjun',
    maintainer_email='qndks11@gmail.com',
    description='rqt button panel for the HYPER stack operator services.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hyper_panel = hyper_rqt.main:main',
        ],
    },
)
