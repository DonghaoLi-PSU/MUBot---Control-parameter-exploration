import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mubot_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Donghao Li',
    maintainer_email='donghao.gerald.li@gmail.com',
    description='μBot gait odometry, EKF configuration and landmark pose-graph SLAM.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_odometry = mubot_localization.gait_odometry_node:main',
            'slam = mubot_localization.slam_node:main',
        ],
    },
)
