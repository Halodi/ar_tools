from launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='ar_tools',
            node_executable='calibration_motion_head',
            name='calibration_motion_head',
            output='screen'
        ),
        
        Node(
            package='ar_tools',
            node_executable='calibration_extrinsic_head',
            name='calibration_extrinsic_head',
            output='screen',
            on_exit = launch.actions.Shutdown(),
            arguments = [ '/home/halodi/eve_ws/src/ar_tools/config/extrinsic_calibration.json' ]            
        )
    ])
