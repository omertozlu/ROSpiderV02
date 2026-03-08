import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paket dizinini alıyoruz
    pkg_share = get_package_share_directory('hexapod_teleop_joystick')

    return LaunchDescription([
        Node(
            package='hexapod_teleop_joystick',
            executable='hexapod_teleop_joystick',
            name='hexapod_teleop_joystick',
            output='screen',
            parameters=[{
                'STANDUP_BUTTON': 0,
                'SITDOWN_BUTTON': 1,
                'BODY_ROTATION_BUTTON': 2,
                'FORWARD_BACKWARD_AXES': 1,
                'LEFT_RIGHT_AXES': 0,
                'YAW_ROTATION_AXES': 2,
                'PITCH_ROTATION_AXES': 3,
                'MAX_METERS_PER_SEC': 0.5,
                'MAX_RADIANS_PER_SEC': 1.0,
            }]
        )
    ])