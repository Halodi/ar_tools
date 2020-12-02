import rclpy, tf2_ros
from rclpy.qos import *
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from halodi_msgs.msg import ARMarkers, ExtrinsicCalibrationInfo

from time import perf_counter
from scipy.optimize import differential_evolution
import numpy as np
from ar_tools.transforms_math import *
from ar_tools.ArMarkerDataContainer import *
from ar_tools.io import save_calib_data, load_calib_data

class ExtrinsicCalibrationBase(rclpy.node.Node):
    def __init__(self, cfg, de_bounds):
        super().__init__('extrinsic_calibration')
         
        self._cfg = cfg
        self._cfg['de']['func'] = self.stationary_target_error_fn
        self._cfg['de']['bounds'] = de_bounds
        self._tf_buffer_core = None
        self._optimization_result = None

        self._ar_marker_data = ArMarkerDataContainer()
        self._tf_msgs = []
        self._tf_static_msgs = []

    def tf_cb(self, msg):
        self._tf_msgs.extend(msg.transforms)

    def tf_static_cb(self, msg):
        self._tf_static_msgs.extend(msg.transforms)
            
    def markers_cb(self, msg): 
        frame_ids_and_fms_ = []       
        for marker in msg.markers:
            if marker.pose.header.frame_id in self._cfg['stationary_target_frames']:
                fm_ = FramedMatrix(msg.header.frame_id, pose_to_matrix(marker.pose.pose))
                frame_ids_and_fms_.append([ marker.pose.header.frame_id, fm_ ])
                
        ns_ = (msg.header.stamp.sec * int(1e9)) + msg.header.stamp.nanosec
        self._ar_marker_data.append(ns_, frame_ids_and_fms_)
                
    def collect_data(self):
        self.get_logger().info("Collecting data ...")
        
        def spin_for(dt):
            end_time_ = perf_counter() + dt
            while perf_counter() < end_time_:
                rclpy.spin_once(self)

        tf_sub_ = self.create_subscription(TFMessage, "/tf", self.tf_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        tf_static_sub_ = self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST))
        spin_for(self._cfg['tf_bookend_duration'])
        
        ar_sub_ = self.create_subscription(ARMarkers, self._cfg['markers_topic'], self.markers_cb, 10)
        spin_for(self._cfg['data_collection_duration'])
        self.destroy_subscription(ar_sub_)
        
        spin_for(self._cfg['tf_bookend_duration'])
        self.destroy_subscription(tf_sub_)
        self.destroy_subscription(tf_static_sub_)
        
        self._ar_marker_data.downsample(self._cfg['data_collection_samples_n'])
        
        if self._cfg['data_save_folder'] != "":
            save_calib_data(self._cfg['data_save_folder'], self._ar_marker_data, self._tf_msgs, self._tf_static_msgs)
            self.get_logger().info("Saved data to " + self._cfg['data_save_folder'])
            
        self.get_logger().info("Data collection finished with " + str(len(self._ar_marker_data)) + " marker samples")

    def load_data(self):
        [ self._ar_marker_data, self._tf_msgs, self._tf_static_msgs ] = load_calib_data(self._cfg['data_save_folder'])
        
    def get_static_transform_matrix(self, x):
        return np.eye(4)
        
    def get_camera_frame_adjustment_matrix(self, x):
        return self.get_static_transform_matrix(x)
        
    def stationary_target_error_fn(self, x):
        timeshift_ns_ = int(x[0] * 1e9)
        camera_frame_adjustment_matrix_ = self.get_camera_frame_adjustment_matrix(x)                
        projections_ = {}
        for frame_id in self._cfg['stationary_target_frames']: projections_[frame_id] = []
            
        for ns,markers in self._ar_marker_data.generator():
            try:
                ts_ = timestamp_nanoseconds(ns - timeshift_ns_)
                wcp_stf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['static_frame'], self._cfg['camera_frame_parent'], ts_)
                wc_mat_ = np.matmul(transform_to_matrix(wcp_stf_.transform), camera_frame_adjustment_matrix_)
            except Exception as e:
                self.get_logger().error(str(e))
                continue
            
            for frame_id,fm in markers:
                if frame_id in projections_.keys():
                    wt_mat_ = np.matmul(wc_mat_, fm.M)
                    pos_ = wt_mat_[:3,3].tolist()
                    if self._cfg['project_rotations']: pos_.extend(np.sum(wt_mat_[:3,:3], axis=1).tolist())
                    projections_[frame_id].append(pos_)
                else: self.get_logger().error("Got invalid marker frame_id " + frame_id)
                
        std_sums_all_ = sum([ np.std(np.asarray(v), axis=0).sum() for v in projections_.values() if len(v) >= self._cfg['marker_opt_count_threshold'] ])

        return std_sums_all_ * std_sums_all_
            
    def optimize(self):
        self.get_logger().info("Populating TF buffer core ...")
        self._tf_buffer_core = tf2_ros.BufferCore(Duration(seconds=600))
        who_ = 'default_authority'
        for tf_msg in self._tf_msgs: self._tf_buffer_core.set_transform(tf_msg, who_)
        for tf_msg in self._tf_static_msgs: self._tf_buffer_core.set_transform_static(tf_msg, who_)

        self.get_logger().info("Transforming marker matrices ...")
        for ns,markers in self._ar_marker_data.generator():
            ts_ = timestamp_nanoseconds(ns)
            for fm in markers.values():
                if fm.parent_frame_id == desired_parent_id: continue                
                try:
                    stf_ = self._tf_buffer_core.lookup_transform_core(self._cfg['camera_frame'], fm.parent_frame_id, ts_)
                    fm.M = np.matmul(transform_to_matrix(stf_.transform), fm.M)
                    fm.parent_frame_id = self._cfg['camera_frame']
                except Exception as e: self.get_logger().error(str(e))

        self.get_logger().info("Optimizing ...")
        res_ = differential_evolution(**self._cfg['de'])
        
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
        
    def run(self, cmd_str):
        if "collect" in cmd_str: self.collect_data()
        elif "load"  in cmd_str: self.load_data()
        else:
            self.get_logger().error("No data input method (collect or load) specified in " + cmd_str)
            return

        if "optimize" in cmd_str: self.optimize()
        if "publish"  in cmd_str: self.publish_extrinisic_calibration_info()

