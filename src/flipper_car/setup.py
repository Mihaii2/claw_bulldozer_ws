import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'flipper_car'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mihai',
    maintainer_email='user@todo.todo',
    description='Flipper RC Car',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_controller = flipper_car.esp32_controller:main',
            'teleop_keyboard = flipper_car.teleop_keyboard:main',
            'trajectory_tracker = flipper_car.trajectory_tracker:main',
        ],
    },
)