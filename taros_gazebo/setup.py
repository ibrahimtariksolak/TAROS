import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'taros_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models', 'quadrotor'),
            glob('models/quadrotor/*')),
        (os.path.join('share', package_name, 'models', 'fixedwing'),
            glob('models/fixedwing/*')),
        (os.path.join('share', package_name, 'models', 'hexarotor'),
            glob('models/hexarotor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik Solak',
    maintainer_email='ibrahimtariksolak@users.noreply.github.com',
    description='TAROS Gazebo worlds, drone models, spawner and pilot nodes',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'drone_spawner = taros_gazebo.drone_spawner_node:main',
            'drone_pilot = taros_gazebo.drone_pilot_node:main',
            'scenario_manager = taros_gazebo.scenario_manager_node:main',
        ],
    },
)
