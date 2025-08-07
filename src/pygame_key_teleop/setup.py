from setuptools import setup

package_name = 'pygame_key_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f1',
    maintainer_email='joonbro0116@naver.com',
    description='TODO: Package description',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pygame_key_teleop = pygame_key_teleop.teleop_node:main',
        ],
    },
)
