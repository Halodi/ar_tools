from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_tools',
            node_executable='aruco_grpc',
            name='aruco_grpc',
            output='screen',
            arguments = [ '/home/james/eloquent_ws/src/ar_tools/config.json', '10.4.2.30:11311', '10.4.2.30:11310' ]
        ),
        
        Node(
            package='ar_tools',
            node_executable='stf_server',
            name='stf_server',
            output='screen'
        )
    ])
