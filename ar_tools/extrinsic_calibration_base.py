import rclpy, tf2_ros
from halodi_msgs.msg import ARMarkers
from geometry_msgs.msg import Transform

from time import monotonic
from scipy.optimize import minimize
import numpy as np
from ar_tools.transforms_math import transform_to_matrix

class ExtrinsicCalibrationBase(rclpy.node.Node):
    def __init__(self, cfg, x0):
        super().__init__('extrinsic_calibration')
         
        self._cfg = cfg
        self._cfg['x0_'] = x0
        self.optimization_result = None
        
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self,
            qos = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT),
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE))
            
        self.create_subscription = node.create_subscription(ARMarkers, cfg['markers_topic'], self.ar_cb, 10)
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
        
    def aggregate_data(self):
        self.get_logger().info("Collecting data for " + str(self._cfg['data_aggregation_duration']) + " seconds ...")
        
        end_time_ = monotonic() + self._cfg['data_aggregation_duration']
        while monotonic() < end_time_: rclpy.spin_once(self)
            
        self.get_logger().info("Data collection finished (" + str(len(self._ar_msgs)) + " target samples)")
        
    def get_camera_frame_adjustment_matrix_fn(x):
        return np.eye(4)
        
    def clip_camera_delay(x):
        return np.clip(x[0], self._cfg['camera_delay_min'], self._cfg['camera_delay_max'])
        
    def get_camera_delay_duration(x):
        delay_ = self.clip_camera_delay(x)
        s_ = int(delay_)
        ns_ = int((delay_ - s_) * 1e9)
        return Duration(seconds=s_, nanoseconds=ns_)        
        
    def static_target_error_fn(self, x):
        camera_delay_ = get_camera_delay_duration(x)
        camera_frame_adjustment_matrix_ = self.get_camera_frame_adjustment_matrix_fn(x)        
        
        fixed_frame_target_matrices_ = np.empty([ len(self._ar_msgs), 4, 4 ])
        I_ = []
        
        for i in range(len(self._ar_msgs)):            
            try:
                ts_ = self._ar_msgs[i][0] - camera_delay_
                wc_stf_ = self._tf_buffer.lookup_transform(target_frame=self._cfg['static_frame'], source_frame=self._cfg['camera_frame_parent'], time=ts_)
                wc_mat_ = np.matmul(transform_to_matrix(wc_stf_.transform), camera_frame_adjustment_matrix_)
                fixed_frame_target_matrices_[i] = np.matmul(wc_mat_, self._ar_msgs[i][1])
                I_.append(i)
            except: continue
            
        xvar_ = np.var(fixed_frame_target_matrices_[I_,0,3])
        yvar_ = np.var(fixed_frame_target_matrices_[I_,1,3])
        zvar_ = np.var(fixed_frame_target_matrices_[I_,2,3])
        
        return xvar_ + yvar_ + zvar_
            
    def optimize(self):
        self.get_logger().info("Optimizing ...")
        self.optimization_result = minimize(self.static_target_error_fn, self._cfg['x0_'])
        
        if self.optimization_result.success: self.get_logger().info("Optimization successful. x: " + str(self.optimization_result.x))
        else: self.get_logger().error("Optimization failed")
        
        return self.optimization_result
    
    def aggregate_data_and_optimize(self):
        self.aggregate_data()
        return self.optimize()

