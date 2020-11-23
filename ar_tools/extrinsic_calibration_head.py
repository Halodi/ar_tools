import sys, json
import numpy as np
from scipy.spatial.transform import Rotation
from ar_tools.extrinsic_calibration_base import ExtrinsicCalibrationBase

from halodi_msgs.msg import ExtrinsicCalibrationInfo, RobotJointCalibrationInfo
from geometry_msgs.msg import TransformStamped, Transform

class ExtrinsicCalibrationHead(ExtrinsicCalibrationBase):
    def __init__(config_file):
        with open(config_file, 'r') as f:
            config_ = json.load(f)
        
        x0_ = [ config_['head']['camera_delay'], config_['head']['neck_pitch_offset'] ]
        x0_.extend(config_['head']['neck_to_camera_xyz_ypr'])
        
        super().__init__(config_['common'], x0_)
        
    def get_camera_frame_adjustment_matrix(self, x):
        neck_pitch_matrix_ = np.eye(4)
        neck_pitch_matrix_[:3,:3] = Rotation.from_rotvec([ 0, x[1], 0 ]).as_matrix()
        
        neck_camera_matrix_ = np.empty([4,4])
        neck_camera_matrix_[3,:] = [ 0, 0, 0, 1 ]
        neck_camera_matrix_[:3,3] = x[2:5]
        neck_camera_matrix_[:3,:3] = Rotation.from_euler('zyx', x[5:8])
        
        return np.matmul(neck_pitch_matrix_, neck_camera_matrix_)
        
    def populate_outbound_calibration_msg(self, x):
        out_ = ExtrinsicCalibrationInfo()
        
        out_.joint_infos.append(RobotJointCalibrationInfo(frame_id=self._cfg['camera_frame_parent'], transmission_ratio=1.0, measurement_offset=x[1]))
        
        ctf_ = Transform()
        ctf_.transform.translation.x = x[2]
        ctf_.transform.translation.y = x[3]
        ctf_.transform.translation.z = x[4]
        quat_ = Rotation.from_euler('zyx', x[5:8]).as_quat()
        ctf_.rotation.x = quat_[0]
        ctf_.rotation.y = quat_[1]
        ctf_.rotation.z = quat_[2]
        ctf_.rotation.w = quat_[3]
        
        sctf_ = TransformStamped(child_frame_id=self._cfg['camera_name'], transform=ctf_)
        sctf_.header.frame_id = self._cfg['camera_frame_parent']
        out_.sensor_transforms.append(sctf_)
        
        return out_
        
def main(args=None):
    rclpy.init(args=args)
    
    head_calibrator_ = ExtrinsicCalibrationHead(sys.argv[1])
    if head_calibrator_.aggregate_data_and_optimize():
        head_calibrator_.publish_extrinsic_calibration_info()
    
    head_calibrator_.destroy_node()
    rclpy.shutdown()
