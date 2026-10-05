import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mubot_control'

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
    description='μBot swimming primitives, gait generator, behaviour and heading control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_generator = mubot_control.gait_generator_node:main',
            'behavior = mubot_control.behavior_node:main',
            'twist_to_primitive = mubot_control.twist_to_primitive:main',
        ],
    },
)
