import sys, json
import numpy as np
from scipy.spatial.transform import Rotation
from ar_tools.extrinsic_calibration_base import ExtrinsicCalibrationBase

class ExtrinsicCalibrationHead(ExtrinsicCalibrationBase):
    def __init__(config_file):
        with open(config_file, 'r') as f:
            config_ = json.load(f)
        
        x0_ = [ config_['head']['camera_delay'], config_['head']['neck_pitch_offset'] ]
        x0_.extend(config_['head']['neck_to_camera_xyz_ypr'])
        
        super().__init__(config_['common'], x0_)
        
    def get_camera_frame_adjustment_matrix_fn(self, x):
        neck_pitch_matrix_ = np.eye(4)
        neck_pitch_matrix_[:3,:3] = Rotation.from_rotvec([ 0, x[1], 0 ]).as_matrix()
        
        neck_camera_matrix_ = np.empty([4,4])
        neck_camera_matrix_[3,:] = [ 0, 0, 0, 1 ]
        neck_camera_matrix_[:3,3] = x[2:5]
        neck_camera_matrix_[:3,:3] = Rotation.from_euler('zyx', x[5:8])
        
        return np.matmul(neck_pitch_matrix_, neck_camera_matrix_)
        
def main(args=None):
    rclpy.init(args=args)
    
    head_calibrator_ = ExtrinsicCalibrationHead(sys.argv[1])
    res_ = head_calibrator_.aggregate_data_and_optimize()
    
    head_calibrator_.get_logger().info(str([ res_.success, res_.x ]))
    
    head_calibrator_.destroy_node()
    rclpy.shutdown()
