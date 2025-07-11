from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Dodaj pliki launch i config, jeśli istnieją i chcesz je instalować
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))), # Przykładowo, jeśli masz pliki .yaml w config
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.urdf*')) + glob(os.path.join('description', '*.xacro*'))), # Dodaj pliki opisu robota
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasberka',
    maintainer_email='rasberka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Tutaj dodasz punkty wejścia dla wykonywalnych skryptów Pythona
            # Na przykład: 'my_node = robot.my_node:main'
            'odometry_publisher = robot.odometry_publisher:main',
            #'teleop_keyboard_node = robot.teleop_keyboard_node:main',
        ],
    },
)