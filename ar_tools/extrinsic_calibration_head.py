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
            
        de_bounds_ = []
        for i in range(len(config_['head']['mean'])):
            min_ = config_['head']['mean'][i] - config_['head']['extents'][i]
            max_ = config_['head']['mean'][i] + config_['head']['extents'][i]
            de_bounds_.append(( min_, max_ ))
        
        super().__init__(config_['common'], de_bounds_)
        
    def _get_static_transform_matrix(self, x):
        head_camera_matrix_ = np.empty([4,4])
        head_camera_matrix_[3,:] = [ 0, 0, 0, 1 ]
        head_camera_matrix_[:3,3] = x[2:5]
        head_camera_matrix_[:3,:3] = Rotation.from_euler('zyx', x[5:8]).as_matrix()
        
        return head_camera_matrix_
        
    def _get_camera_frame_adjustment_matrix(self, x):
        head_pitch_matrix_ = np.eye(4)
        head_pitch_matrix_[:3,:3] = Rotation.from_rotvec([ 0, x[1], 0 ]).as_matrix()
        
        return np.matmul(head_pitch_matrix_, self.get_static_transform_matrix(x))

    def get_camera_frame_adjustment_matrix(self, x):
        m_ = np.empty([4,4])
        m_[3,:] = [ 0, 0, 0, 1 ]
        m_[:3,3] = [ 0.0985, -0.032, 0.18 ]
        m_[:3,:3] = Rotation.from_quat([ -0.001, 0.1039, -0.006, 0.994 ]).as_matrix()

        return m_
        
    def get_extrinsic_calibration_info_msg(self, x):
        out_ = super().get_extrinsic_calibration_info_msg(x)
        
        parent_frame_id_ = out_.sensor_transforms[0].header.frame_id
        out_.joint_infos.append(RobotJointCalibrationInfo(frame_id=parent_frame_id_, transmission_ratio=1.0, measurement_offset=x[1]))
        
        return out_
        
def main(args=None):
    rclpy.init(args=args)
    
    head_calibrator_ = ExtrinsicCalibrationHead(sys.argv[1])
    head_calibrator_.collect_data_and_optimize()
#    if head_calibrator_.collect_data_and_optimize():
#        head_calibrator_.publish_extrinsic_calibration_info()
    
    head_calibrator_.destroy_node()
    rclpy.shutdown()
