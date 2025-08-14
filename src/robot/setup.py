from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # rejestracja pakietu w systemie ROS 2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml w share
        ('share/' + package_name, ['package.xml']),
        # folder launch z plikami .py
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # folder config z plikami YAML
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasberka',
    maintainer_email='rasberka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # Tutaj możesz dodać swoje węzły ROS 2 np.:
            # 'nawigowanie_node = robot.nawigowanie_odometria:main',
        ],
    },
)
