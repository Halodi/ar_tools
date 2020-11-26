import rclpy, tf2_ros
from rclpy.qos import *
from geometry_msgs.msg import Transform, TransformStamped
from tf2_msgs.msg import TFMessage
from halodi_msgs.msg import ARMarkers, ExtrinsicCalibrationInfo

from time import perf_counter
from scipy.optimize import minimize
import numpy as np
from ar_tools.transforms_math import *
from ar_tools.throttle import Throttle

class ExtrinsicCalibrationBase(rclpy.node.Node):
    def __init__(self, cfg, x0):
        super().__init__('extrinsic_calibration')
         
        self._cfg = cfg
        self._cfg['x0_'] = x0
        self._optimization_result = None
        
        self._tf_buffer_core = None
        self._ar_stamps_and_tfs = []

    def tf_cb(self, msg):
        who = 'default_authority'
        for tf in msg.transforms:
           self._tf_buffer_core.set_transform(tf, who)              

    def tf_static_cb(self, msg):
        who = 'default_authority'
        for tf in msg.transforms:
            self._tf_buffer_core.set_transform_static(tf, who)
            
    def markers_cb(self, msg):
        for marker in msg.markers:
            if marker.pose.header.frame_id == self._cfg['static_target_frame']:
                ts_ = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                
                tf_ = Transform()
                tf_.translation.x = marker.pose.pose.position.x
                tf_.translation.y = marker.pose.pose.position.y
                tf_.translation.z = marker.pose.pose.position.z
                tf_.rotation.x = marker.pose.pose.orientation.x
                tf_.rotation.y = marker.pose.pose.orientation.y
                tf_.rotation.z = marker.pose.pose.orientation.z
                tf_.rotation.w = marker.pose.pose.orientation.w
                tf_mat_ = transform_to_matrix(tf_)
                
                if msg.header.frame_id == self._cfg['camera_frame']: self._ar_stamps_and_tfs.append([ ts_, tf_mat_ ])
                else: self._ar_stamps_and_tfs.append([ ts_, tf_mat_, msg.header.frame_id ])
                    
                break
                
        
    def aggregate_data(self):
        self.get_logger().info("Collecting data from TF ...")

        self._tf_buffer_core = tf2_ros.BufferCore(Duration(seconds=600))
        tf_sub_ = self.create_subscription(TFMessage, "/tf", self.tf_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        tf_static_sub_ = self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST))

        throttle_ = Throttle(self._cfg['data_aggregation_frequency'])        
        def spin_for(dt):
            end_time_ = perf_counter() + dt
            while perf_counter() < end_time_:
                rclpy.spin_once(self)
                throttle_.wait()            
        
        spin_for(self._cfg['camera_delay_max'] + 5)        
        ar_sub_ = self.create_subscription(ARMarkers, self._cfg['markers_topic'], self.markers_cb, 10)
        spin_for(self._cfg['data_aggregation_duration'])

        self.destroy_subscription(tf_sub_)
        self.destroy_subscription(tf_static_sub_)
        self.destroy_subscription(ar_sub_)
        
        skip_ = int(len(self._ar_stamps_and_tfs) / self._cfg['data_aggregation_samples_n'])
        if skip_ > 1: self._ar_stamps_and_tfs = self._ar_stamps_and_tfs[::skip_]

        for ar_stamp_and_tf in self._ar_stamps_and_tfs:
            if len(ar_stamp_and_tf) == 3:
                wc_tf_ = self._tf_buffer_core.lookup_transform_core(ar_stamp_and_tf[2], self._cfg['camera_frame'], ar_stamp_and_tf[0])
                wc_inv_mat_ = invert_transform_matrix(transform_to_matrix(wc_tf_.transform))
                ar_stamp_and_tf[1] = np.matmul(wc_inv_mat_, ar_stamp_and_tf[1])
            
        self.get_logger().info("Data aggregation finished with " + str(len(self._ar_stamps_and_tfs)) + " marker samples")
        
    def get_static_transform_matrix(self, x):
        return np.eye(4)
        
    def get_camera_frame_adjustment_matrix(self, x):
        return self.get_static_transform_matrix(x)
        
    def get_camera_delay_duration(self, x):
        delay_ = np.clip(x[0], self._cfg['camera_delay_min'], self._cfg['camera_delay_max'])
        s_ = int(delay_)
        ns_ = int((delay_ - s_) * 1e9)
        
        return Duration(seconds=s_, nanoseconds=ns_)
        
    def static_target_error_fn(self, x):
        camera_delay_ = self.get_camera_delay_duration(x)
        camera_frame_adjustment_matrix_ = self.get_camera_frame_adjustment_matrix(x)        
        
        m_ = np.empty([ len(self._ar_stamps_and_tfs), 6 ])
        I_ = []
        
        for i in range(len(self._ar_stamps_and_tfs)):
            try:
                ts_ = self._ar_stamps_and_tfs[i][0] - camera_delay_
                wc_stf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['static_frame'], self._cfg['camera_frame_parent'], ts_)
                wc_mat_ = np.matmul(transform_to_matrix(wc_stf_.transform), camera_frame_adjustment_matrix_)
                wt_mat_ = np.matmul(wc_mat_, self._ar_stamps_and_tfs[i][1])
                
                m_[i,:3] = wt_mat_[:3,3]
                m_[i,3:] = np.sum(wt_mat_[:3,:3], axis=1)
                I_.append(i)
            except Exception as e:
                print(str(e))
                continue

        return np.var(m_[I_,:], axis=0).sum() if len(I_) > 1 else 1e9
            
    def optimize(self):
        self.get_logger().info("Optimizing ...")
        res_ = minimize(self.static_target_error_fn, self._cfg['x0_'])
        
        if res_.success:
            self._optimization_result = res_
            self.get_logger().info("Optimization successful. x: " + str(res_.x))
            return True            
        else:
            self.get_logger().error("Optimization failed")
            return False
    
    def aggregate_data_and_optimize(self):
        self.aggregate_data()
        return self.optimize()
        
    def get_extrinsic_calibration_info_msg(self, x):
        tf_ = matrix_to_transform(self.get_static_transform_matrix(x))        
        stf_ = TransformStamped(child_frame_id=self._cfg['camera_name'], transform=tf_)
        stf_.header.frame_id = self._cfg['camera_frame_parent']

        cam_delay_dur_ = self.get_camera_delay_duration(x).to_msg()
        stf_.header.stamp.sec = cam_delay_dur_.sec
        stf_.header.stamp.nanosec = cam_delay_dur_.nanosec
        
        out_ = ExtrinsicCalibrationInfo()
        out_.sensor_transforms.append(stf_)
        
        return out_
        
    def publish_extrinsic_calibration_info(self):
        if self._optimization_result is None: return
        
        pub_ = self.create_publisher(ExtrinsicCalibrationInfo, self._cfg['outbound_calibration_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
        pub_.publish(self.get_extrinsic_calibration_info_msg(self._optimization_result.x))

