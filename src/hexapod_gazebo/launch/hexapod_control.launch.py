from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Joint State Broadcaster Spawner (Eklem durumlarını okur)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # 2. Hexapod Joint Group Controller Spawner (Tüm bacakları kontrol eder)
    hexapod_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hexapod_joint_group_controller"],
    )

    # NOT: Robot State Publisher buradan silindi çünkü ana dosyada çalışıyor!

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        hexapod_controller_spawner
    ])