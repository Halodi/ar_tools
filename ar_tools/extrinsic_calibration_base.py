import rclpy, tf2_ros
from rclpy.qos import *
from halodi_msgs.msg import ARMarkers, ExtrinsicCalibrationInfo
from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage

from time import monotonic
from scipy.optimize import minimize
import numpy as np
from ar_tools.transforms_math import transform_to_matrix
from ar_tools.throttle import Throttle

class ExtrinsicCalibrationBase(rclpy.node.Node):
    def __init__(self, cfg, x0):
        super().__init__('extrinsic_calibration')
         
        self._cfg = cfg
        self._cfg['x0_'] = x0
        self._optimization_result = None
        self._outbound_calibration_msg = None
        
        self._tf_buffer_core = None 
        self._ar_msgs = []
        
    def ar_cb(self, msg):
        for marker in msg.markers:
            if marker.pose.header.frame_id == self._cfg['static_target_ID']:
                ts_ = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                
                tf_ = Transform()
                tf_.translation.x = marker.pose.pose.position.x
                tf_.translation.y = marker.pose.pose.position.y
                tf_.translation.z = marker.pose.pose.position.z
                tf_.rotation.x = marker.pose.pose.orientation.x
                tf_.rotation.y = marker.pose.pose.orientation.y
                tf_.rotation.z = marker.pose.pose.orientation.z
                tf_.rotation.w = marker.pose.pose.orientation.w
                
                self._ar_msgs.append([ ts_, transform_to_matrix(tf_) ])
                break

    def tf_cb(self, msg):
        who = 'default_authority'
        for tf in msg.transforms:
           self._tf_buffer_core.set_transform(tf, who)

    def tf_static_cb(self, msg):
        who = 'default_authority'
        for tf in msg.transforms:
            self._tf_buffer_core.set_transform_static(tf, who)
        
    def aggregate_data(self):
        self._tf_buffer_core = tf2_ros.BufferCore(Duration(seconds=600))
        tf_sub_ = self.create_subscription(TFMessage, "/tf", self.tf_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT))
        tf_static_sub_ = self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))

        throttle_ = Throttle(self._cfg['data_aggregation_frequency'])

        self.get_logger().info("Firstly subscribing to TF for " + str(self._cfg['tf_listener_warmup_duration']) + " seconds ...")
        end_time_ = monotonic() + self._cfg['tf_listener_warmup_duration']
        while monotonic() < end_time_:
            rclpy.spin_once(self)
            throttle_.wait()

        ar_sub_ = self.create_subscription(ARMarkers, self._cfg['markers_topic'], self.ar_cb, 10)
        self.get_logger().info("Subscribing to TF and marker data for " + str(self._cfg['data_aggregation_duration']) + " seconds ...")
        end_time_ = monotonic() + self._cfg['data_aggregation_duration']
        while monotonic() < end_time_:
            rclpy.spin_once(self)
            throttle_.wait()

        self.destroy_subscription(tf_sub_)
        self.destroy_subscription(tf_static_sub_)
        self.destroy_subscription(ar_sub_)
        
        skip_ = int(len(self._ar_msgs) / self._cfg['data_aggregation_samples_n'])
        if skip_ > 1: self._ar_msgs = self._ar_msgs[::skip_]
            
        self.get_logger().info("Data aggregation finished with " + str(len(self._ar_msgs)) + " marker samples")
        
    def get_camera_frame_adjustment_matrix(self, x):
        return np.eye(4)
        
    def clip_camera_delay(self, x):
        return np.clip(x[0], self._cfg['camera_delay_min'], self._cfg['camera_delay_max'])
        
    def get_camera_delay_duration(self, x):
        delay_ = self.clip_camera_delay(x)
        s_ = int(delay_)
        ns_ = int((delay_ - s_) * 1e9)
        
        return Duration(seconds=s_, nanoseconds=ns_)
        
    def static_target_error_fn(self, x):
        camera_delay_ = self.get_camera_delay_duration(x)
        camera_frame_adjustment_matrix_ = self.get_camera_frame_adjustment_matrix(x)        
        
        m_ = np.empty([ len(self._ar_msgs), 6 ])
        I_ = []
        
        for i in range(len(self._ar_msgs)):
            try:
                ts_ = self._ar_msgs[i][0] - camera_delay_
                wc_stf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['static_frame'], self._cfg['camera_frame_parent'], ts_)
                wc_mat_ = np.matmul(transform_to_matrix(wc_stf_.transform), camera_frame_adjustment_matrix_)
                wt_mat_ = np.matmul(wc_mat_, self._ar_msgs[i][1])
                
                m_[i,:3] = wt_mat_[:3,3]
                m_[i,3:] = np.sum(wt_mat_[:3,:3], axis=1)
                I_.append(i)
            except Exception as e:
                #print(str(e))
                continue

        return np.var(m_[I_,:], axis=0).sum() if len(I_) > 1 else 1e9
            
    def optimize(self):
        self.get_logger().info("Optimizing ...")
        self._optimization_result = minimize(self.static_target_error_fn, self._cfg['x0_'])
        
        if self._optimization_result.success:
            self.get_logger().info("Optimization successful. x: " + str(self._optimization_result.x))
            self._outbound_calibration_msg = self.get_outbound_calibration_msg(self._optimization_result.x)
        else: self.get_logger().error("Optimization failed")
        
        return self._optimization_result.success
    
    def aggregate_data_and_optimize(self):
        self.aggregate_data()
        return self.optimize()
        
    def get_outbound_calibration_msg(self, x):
        return None
        
    def publish_extrinsic_calibration_info(self):
        if self._outbound_calibration_msg is None: return
        
        pub_ = self.create_publisher(ExtrinsicCalibrationInfo, self._cfg['outbound_calibration_topic'],
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
        pub_.publish(self._outbound_calibration_msg)

