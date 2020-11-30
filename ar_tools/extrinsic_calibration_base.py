import rclpy, tf2_ros
from rclpy.qos import *
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from halodi_msgs.msg import ARMarkers, ExtrinsicCalibrationInfo

from time import perf_counter
from scipy.optimize import differential_evolution
import numpy as np
from ar_tools.transforms_math import *
import pickle

class ExtrinsicCalibrationBase(rclpy.node.Node):
    def __init__(self, cfg, de_bounds):
        super().__init__('extrinsic_calibration')
         
        self._cfg = cfg
        self._cfg['de_bounds_'] = de_bounds
        self._optimization_result = None
        
        self._tf_buffer_core = None
        self._ar_stamps_and_tfs = []
        self._tf_msgs = []
        self._tf_static_msgs = []

    def tf_cb(self, msg):
        self._tf_msgs.extend(msg.transforms)

    def tf_static_cb(self, msg):
        self._tf_static_msgs.extend(msg.transforms)
            
    def markers_cb(self, msg):
        for marker in msg.markers:
            if marker.pose.header.frame_id == self._cfg['stationary_target_frame']:
                ts_ = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)
                tf_mat_ = pose_to_matrix(marker.pose.pose)
                
                if msg.header.frame_id == self._cfg['camera_frame']: self._ar_stamps_and_tfs.append([ ts_, tf_mat_ ])
                else: self._ar_stamps_and_tfs.append([ ts_, tf_mat_, msg.header.frame_id ])
                    
                break
                
    def collect_data(self):
        self.get_logger().info("Collecting data ...")

        tf_sub_ = self.create_subscription(TFMessage, "/tf", self.tf_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        tf_static_sub_ = self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST))

        def spin_for(dt):
            end_time_ = perf_counter() + dt
            while perf_counter() < end_time_:
                rclpy.spin_once(self)    
        
        spin_for(self._cfg['tf_bookend_duration'])
        ar_sub_ = self.create_subscription(ARMarkers, self._cfg['markers_topic'], self.markers_cb, 10)
        spin_for(self._cfg['data_collection_duration'])
        self.destroy_subscription(ar_sub_)
        spin_for(self._cfg['tf_bookend_duration'])
        self.destroy_subscription(tf_sub_)
        self.destroy_subscription(tf_static_sub_)
        
        skip_ = int(len(self._ar_stamps_and_tfs) / self._cfg['data_collection_samples_n'])
        if skip_ > 1: self._ar_stamps_and_tfs = self._ar_stamps_and_tfs[::skip_]

        if self._cfg['pickle_file'] != "":
            pickle.dump([ self._ar_stamps_and_tfs, self._tf_msgs, self._tf_static_msgs ], open(self._cfg['pickle_file'], 'wb'))
            self.get_logger().info("Saved data to " + self._cfg['pickle_file'])
            
        self.get_logger().info("Data collection finished with " + str(len(self._ar_stamps_and_tfs)) + " marker samples")

    def load_data(self):
        [ self._ar_stamps_and_tfs, self._tf_msgs, self._tf_static_msgs ] = pickle.load(open(self._cfg['pickle_file'], 'rb'))
        
    def get_static_transform_matrix(self, x):
        return np.eye(4)
        
    def get_camera_frame_adjustment_matrix(self, x):
        return self.get_static_transform_matrix(x)
        
    def stationary_target_error_fn(self, x):
        camera_frame_adjustment_matrix_ = self.get_camera_frame_adjustment_matrix(x)        
        
        width_ = 6 if self._cfg['project_rotations'] else 3
        m_ = np.empty([ len(self._ar_stamps_and_tfs), width_ ])
        I_ = []
        
        for i in range(len(self._ar_stamps_and_tfs)):
            try:
                ts_ = rclpy.time.Time(nanoseconds=self._ar_stamps_and_tfs[i][0].nanoseconds - int(x[0] * 1e9))
                wc_stf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['static_frame'], self._cfg['camera_frame_parent'], ts_)
                wc_mat_ = np.matmul(transform_to_matrix(wc_stf_.transform), camera_frame_adjustment_matrix_)
                wt_mat_ = np.matmul(wc_mat_, self._ar_stamps_and_tfs[i][1])
                
                m_[i,:3] = wt_mat_[:3,3]
                if self._cfg['project_rotations']: m_[i,3:] = np.sum(wt_mat_[:3,:3], axis=1)
                I_.append(i)
            except Exception as e: self.get_logger().error(str(e))
                
        if len(I_) > 1:
            std_ = np.std(m_[I_,:], axis=0)
            if self._cfg['verbose_optimization']: self.get_logger().info(str(std_))
            std_sum_ = std_.sum()
            return std_sum_ * std_sum_
        else: return 1e9
            
    def optimize(self):
        self.get_logger().info("Populating TF buffer ...")
        self._tf_buffer_core = tf2_ros.BufferCore(Duration(seconds=600))
        who_ = 'default_authority'
        for tf_msg in self._tf_msgs: self._tf_buffer_core.set_transform(tf_msg, who_)
        for tf_msg in self._tf_static_msgs: self._tf_buffer_core.set_transform_static(tf_msg, who_)

        for ar_stamp_and_tf in self._ar_stamps_and_tfs:
            if len(ar_stamp_and_tf) == 3:
                cw_tf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['camera_frame'], ar_stamp_and_tf[2], ar_stamp_and_tf[0])
                ar_stamp_and_tf[1] = np.matmul(transform_to_matrix(cw_tf_.transform), ar_stamp_and_tf[1])
                ar_stamp_and_tf.pop()

        self.get_logger().info("Optimizing ...")
        res_ = differential_evolution(self.stationary_target_error_fn, self._cfg['de_bounds_'],
            strategy      = self._cfg['de'].get('strategy', 'best1bin'),
            maxiter       = self._cfg['de'].get('maxiter', 1000),
            popsize       = self._cfg['de'].get('popsize', 15),
            atol          = self._cfg['de'].get('atol', 0.0),
            tol           = self._cfg['de'].get('tol', 0.01),
            mutation      = self._cfg['de'].get('mutation', 0.5),
            recombination = self._cfg['de'].get('recombination', 0.7),
            polish        = self._cfg['de'].get('polish', True),
            workers       = self._cfg['de'].get('workers', 1))
        
        if res_.success:
            self._optimization_result = res_
            self.get_logger().info("Optimization successful. x: " + str(res_.x))
            return True            
        else:
            self.get_logger().error("Optimization failed")
            return False
        
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

