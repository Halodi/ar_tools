from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ar_tools',
            node_executable='aruco_grpc',
            name='aruco_grpc',
            output='screen',
            arguments = [ '/home/halodi/eve_ws/src/ar_tools/config/aruco.json', '10.4.2.30:30000', '10.4.2.30:30001' ]
        ),
        
        Node(
            package='ar_tools',
            node_executable='stf_server',
            name='stf_server',
            output='screen',
        )
    ])
