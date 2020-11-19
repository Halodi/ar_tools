from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_tools',
            node_executable='aruco_zed',
            name='aruco_zed',
            output='screen',
            arguments = [ '10.4.2.30:30000' ]
        ),
        
        Node(
            package='ar_tools',
            node_executable='stf_server',
            name='stf_server',
            output='screen'
        )
    ])
