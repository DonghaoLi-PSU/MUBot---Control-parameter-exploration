import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mubot_learning'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'cases'), glob('config/cases/*.yaml')),
        (os.path.join('share', package_name, 'policies'), glob('policies/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Donghao Li',
    maintainer_email='donghao.gerald.li@gmail.com',
    description='EM-PGPE (EPHE) gait optimization for μBot with parallel Gazebo workers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trainer = mubot_learning.trainer:main',
        ],
    },
)
