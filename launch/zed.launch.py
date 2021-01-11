from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_tools',
            node_executable='aruco_zed',
            name='aruco_zed_left',
            output='screen',
            arguments = [ '/home/halodi/ros2_ws/src/ar_tools/config/aruco.json', '10.4.1.30:30000', 'left' ]
        ),

        Node(
            package='ar_tools',
            node_executable='aruco_zed',
            name='aruco_zed_right',
            output='screen',
            arguments = [ '/home/halodi/ros2_ws/src/ar_tools/config/aruco_right.json', '10.4.1.30:30000', 'right' ]
        ),
        
        Node(
            package='ar_tools',
            node_executable='stf_server',
            name='stf_server',
            output='screen',
        )
    ])
