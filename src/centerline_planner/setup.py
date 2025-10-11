from setuptools import setup

package_name = 'centerline_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/centerline_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moon',
    maintainer_email='moon@example.com',
    description='Simple global centerline publisher and pure pursuit follower',
    license='MIT',
    entry_points={
        'console_scripts': [
            'centerline_pub = centerline_planner.centerline_pub:main',
            'path_pure_pursuit = centerline_planner.path_pure_pursuit:main',
        ],
    },
)
