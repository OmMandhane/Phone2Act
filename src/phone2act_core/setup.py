import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'phone2act_core'

setup(
    name=package_name,
    version='0.0.1',
    # find_packages handles finding the inner folder automatically
    packages=find_packages(exclude=['test']),
    data_files=[
        # 1. CRITICAL: Standard ROS 2 Package Registration
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        ('share/' + package_name, ['package.xml']),
        
        # 2. FIX: Install all launch files dynamically
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 3. FIX: Install all config/yaml files (Required for your launch file!)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='om',
    maintainer_email='ommandhane27@gmail.com',
    description='Hardware-agnostic core logic for Phone2Act',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phone2act_teleop_planner = phone2act_core.phone2act_teleop_planner:main',
            'universal_recorder = phone2act_core.phone2act_universal_recorder:main',
        ],
    },
)