import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('hexapod_description')
    
    # DOSYA ADINI BURADA SABİTLEDİK (Sidebar'daki isimle birebir aynı)
    xacro_file = os.path.join(pkg_share, 'urdf', 'hexapod_model.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'hexapod.rviz')]
        )
    ])