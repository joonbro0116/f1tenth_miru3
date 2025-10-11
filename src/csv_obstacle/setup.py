from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'csv_obstacle'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Publish obstacles from CSV (inner/outer bounds)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'csv_obstacle = csv_obstacle.node:main',
        ],
    },
)
