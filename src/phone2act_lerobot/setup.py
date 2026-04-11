import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'phone2act_lerobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='om',
    maintainer_email='ommandhane27@gmail.com',
    description='LeRobot hardware bridge for Phone2Act',
    license='MIT', 
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lerobot_hardware_bridge = phone2act_lerobot.lerobot_hardware_bridge:main',
        ],
    },
)