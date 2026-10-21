"""TAROS tam sistem baslatmasi: simuelasyon + tum alt sistemler."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """Tam sistem launch tanimini olusturur."""
    pkg_gazebo = get_package_share_directory('taros_gazebo')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch',
                         'taros_sim.launch.py')),
        launch_arguments={
            'scenario': LaunchConfiguration('scenario'),
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='single_recon',
            description='Tehdit senaryosu'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Gazebo arayuzu'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Ayrintili gunluk'),
        sim_launch,
    ])
