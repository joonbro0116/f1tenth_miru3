from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'forza_map_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/steering_lut'), glob('config/steering_lut/*.csv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='sh',
    maintainer_email='akdrhaktapfhs1@gmail.com',
    description='Forza MAP Controller - ported from race_stack to ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_manager = forza_map_controller.controller_manager:main',
        ],
    },
)
