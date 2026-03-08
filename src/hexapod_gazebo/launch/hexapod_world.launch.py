import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paket yollarını al (Senin klasör yapına göre güncelledim)
    pkg_gazebo = get_package_share_directory('hexapod_gazebo')
    pkg_description = get_package_share_directory('hexapod_description')

    # 2. Xacro dosyasını işle (İsim düzeltildi: hexapod_model.xacro)
    xacro_file = os.path.join(pkg_description, 'urdf', 'hexapod_model.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    # 3. Gazebo'yu başlat (Server ve Client)
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        )
    )
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    )

    # 4. Robot State Publisher (URDF yayıncı)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 5. Robotu Gazebo'ya yerleştir (Entity: Golem)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'Golem', '-z', '0.2'],
        output='screen'
    )

    # 6. Kontrolcüleri (Spawners) başlat
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # İsim phantomx_control.yaml ile eşitlendi
    hexapod_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hexapod_joint_group_controller"],
    )

    # 7. Gecikmeli Başlatma (Fizik motorunun oturması için 5 sn bekle)
    delayed_controllers = TimerAction(
        period=5.0,
        actions=[
            joint_state_broadcaster,
            hexapod_controller
        ]
    )

    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        node_robot_state_publisher,
        spawn_entity,
        delayed_controllers
    ])