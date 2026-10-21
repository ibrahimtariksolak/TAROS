"""Kurulum betigi."""
from setuptools import find_packages, setup

package_name = 'taros_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarik Solak',
    maintainer_email='ibrahimtariksolak@users.noreply.github.com',
    description='TAROS UGV otonom intikal ve mevzi secimi',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'navigation = taros_navigation.navigation_node:main',
        ],
    },
)
