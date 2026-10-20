"""
TAROS simulasyonunu baslatir: Gazebo + hedef uretimi + ucus kontrolu + sensorler.

gazebo.launch.py yerine gzserver/gzclient dosyalari dogrudan include edilir:
gazebo.launch.py alt launch dosyalarina launch_arguments iletmez, bu yuzden
world gibi argumanlarin gectigine guvenmek kirilgandir.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCENARIOS = (
    'single_recon', 'dual_probe', 'swarm_wave', 'swarm_saturate', 'mixed_loiter')


def generate_launch_description():
    pkg_taros_gazebo = get_package_share_directory('taros_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_taros_gazebo, 'worlds', 'defense_base.world')
    model_path = os.path.join(pkg_taros_gazebo, 'models')

    # Gazebo model://... cozumlemesi icin model yolu sürece aktarilmali.
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    set_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        model_path + (os.pathsep + existing_model_path if existing_model_path else ''))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world_file,
            'verbose': LaunchConfiguration('verbose'),
            # factory: /spawn_entity, init: /clock. Ikisi de gerekli.
            'init': 'true',
            'factory': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    drone_spawner = Node(
        package='taros_gazebo',
        executable='drone_spawner',
        name='drone_spawner',
        output='screen',
        emulate_tty=True,
    )

    drone_pilot = Node(
        package='taros_gazebo',
        executable='drone_pilot',
        name='drone_pilot',
        output='screen',
        emulate_tty=True,
        parameters=[{'update_rate': LaunchConfiguration('update_rate')}],
    )

    scenario_manager = Node(
        package='taros_gazebo',
        executable='scenario_manager',
        name='scenario_manager',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'scenario': LaunchConfiguration('scenario'),
            'delay_between_spawns': LaunchConfiguration('spawn_delay'),
        }],
    )

    weather_node = Node(
        package='taros_environment',
        executable='weather_node',
        name='weather_node',
        output='screen',
        emulate_tty=True,
    )

    radar_sim = Node(
        package='taros_sensor',
        executable='radar_sim',
        name='radar_sim',
        output='screen',
        emulate_tty=True,
    )

    eoir_camera = Node(
        package='taros_sensor',
        executable='eoir_camera',
        name='eoir_camera',
        output='screen',
        emulate_tty=True,
    )

    acoustic_sim = Node(
        package='taros_sensor',
        executable='acoustic_sim',
        name='acoustic_sim',
        output='screen',
        emulate_tty=True,
    )

    tracker = Node(
        package='taros_tracker',
        executable='tracker',
        name='tracker',
        output='screen',
        emulate_tty=True,
    )

    classifier = Node(
        package='taros_classifier',
        executable='classifier',
        name='classifier',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='single_recon',
            description=f'Tehdit senaryosu. Secenekler: {", ".join(SCENARIOS)}'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Gazebo arayuzunu baslat (headless icin false)'),
        DeclareLaunchArgument(
            'verbose', default_value='false',
            description='Gazebo ayrintili gunlugu'),
        DeclareLaunchArgument(
            'spawn_delay', default_value='2.0',
            description='Hedefler arasi spawn gecikmesi [s]'),
        DeclareLaunchArgument(
            'update_rate', default_value='30.0',
            description='Ucus kontrol dongusu frekansi [Hz]'),

        set_model_path,
        gzserver,
        gzclient,
        drone_spawner,
        drone_pilot,
        scenario_manager,
        weather_node,
        radar_sim,
        eoir_camera,
        acoustic_sim,
        tracker,
        classifier,
    ])
