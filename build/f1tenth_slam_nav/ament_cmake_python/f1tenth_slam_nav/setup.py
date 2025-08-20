from setuptools import find_packages
from setuptools import setup

setup(
    name='f1tenth_slam_nav',
    version='1.0.0',
    packages=find_packages(
        include=('f1tenth_slam_nav', 'f1tenth_slam_nav.*')),
)
