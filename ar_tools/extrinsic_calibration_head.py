import sys, json
import numpy as np
from scipy.spatial.transform import Rotation
from ar_tools.extrinsic_calibration_base import ExtrinsicCalibrationBase

import rclpy
from halodi_msgs.msg import RobotJointCalibrationInfo

class ExtrinsicCalibrationHead(ExtrinsicCalibrationBase):
    def __init__(self, config_file):
        with open(config_file, 'r') as f:
            config_ = json.load(f)
        
        x0_ = [ config_['head']['camera_delay'], config_['head']['pitch_offset'] ]
        x0_.extend(config_['head']['head_to_camera_xyz_ypr'])
        
        super().__init__(config_['common'], x0_)
        
    def get_static_transform_matrix(self, x):
        head_camera_matrix_ = np.empty([4,4])
        head_camera_matrix_[3,:] = [ 0, 0, 0, 1 ]
        head_camera_matrix_[:3,3] = x[2:5]
        head_camera_matrix_[:3,:3] = Rotation.from_euler('zyx', x[5:8]).as_matrix()
        
        return head_camera_matrix_
        
    def get_camera_frame_adjustment_matrix(self, x):
        head_pitch_matrix_ = np.eye(4)
        head_pitch_matrix_[:3,:3] = Rotation.from_rotvec([ 0, x[1], 0 ]).as_matrix()
        
        return np.matmul(head_pitch_matrix_, self.get_static_transform_matrix(x))        
        
    def get_outbound_calibration_msg(self, x):
        out_ = super().get_outbound_calibration_msg(x)
        
        parent_frame_id_ = out_.sensor_transforms[0].header.frame_id
        out_.joint_infos.append(RobotJointCalibrationInfo(frame_id=parent_frame_id_, transmission_ratio=1.0, measurement_offset=x[1]))
        
        return out_
        
def main(args=None):
    rclpy.init(args=args)
    
    head_calibrator_ = ExtrinsicCalibrationHead(sys.argv[1])
    if head_calibrator_.aggregate_data_and_optimize():
        head_calibrator_.publish_extrinsic_calibration_info()
    
    head_calibrator_.destroy_node()
    rclpy.shutdown()
