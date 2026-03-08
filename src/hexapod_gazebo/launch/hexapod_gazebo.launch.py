import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paket yollarını al
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    description_dir = get_package_share_directory('hexapod_description')
    gazebo_pkg_dir = get_package_share_directory('hexapod_gazebo')

    # 2. Xacro ile robot modelini işle
    xacro_file = os.path.join(description_dir, 'urdf', 'hexapod_model.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # 3. Gazebo'yu başlat (pause=false yaparak siyah ekran donmasını engelle)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'pause': 'false'}.items()
    )

    # 4. Robot State Publisher (Simülasyon zamanı açık)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )

    # 5. Robotu Gazebo'ya yerleştir (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'Golem', '-topic', 'robot_description', '-z', '0.15'],
        output='screen'
    )

    # 6. Kontrolcüleri (hexapod_control.launch.py) 5 saniye gecikmeyle başlat
    # Bu gecikme Gazebo plugin'inin yüklenmesine zaman tanır
    load_joint_controllers = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(gazebo_pkg_dir, 'launch', 'hexapod_control.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_controllers
    ])