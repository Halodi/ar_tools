import sys, json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from threading import Thread
from queue import Queue
from time import monotonic, sleep
from os.path import isfile

import rclpy, tf2_ros
from ar_tools.transforms_math import multiply_transforms
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from halodi_msgs.msg import ARMarker, ARMarkers
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock



marker_rot_adj = Rotation.from_rotvec([0, np.pi, 0])

def get_aruco_markers(image_grayscale, cfg):
    corners, ids, _rejectedImgPoints = cv2.aruco.detectMarkers(image_grayscale, cfg['aruco_dict_'], parameters=cfg['aruco_params_'])
    msgs_ = []
    for i in range(len(corners)):
        id_ = str(ids[i][0])
        if id_ not in cfg['marker_sizes'].keys(): continue
        
        msg_ = ARMarker(data="")
        msg_.pose.header.frame_id = id_
        for corner in corners[i].reshape([4,2]):
            msg_.points.append(Point(x=float(corner[0]), y=float(corner[1]), z=0.0))
            
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], cfg['marker_sizes'][id_], cameraMatrix=cfg['K_'], distCoeffs=cfg['d_'])
        
        tvecs_flat_ = tvecs.ravel()
        msg_.pose.pose.position.x =  tvecs_flat_[2]
        msg_.pose.pose.position.y = -tvecs_flat_[0]
        msg_.pose.pose.position.z = -tvecs_flat_[1]
        
        rvecs_flat_ = rvecs.ravel()
        rot_ = Rotation.from_rotvec([ rvecs_flat_[2], -rvecs_flat_[0], -rvecs_flat_[1] ]) * marker_rot_adj
        quat_ = rot_.as_quat()
        msg_.pose.pose.orientation.x = quat_[0]
        msg_.pose.pose.orientation.y = quat_[1]
        msg_.pose.pose.orientation.z = quat_[2]
        msg_.pose.pose.orientation.w = quat_[3]
        msgs_.append(msg_)

    return msgs_
    
class ArucoBroadcaster:
    def __init__(self, rclpy_args, get_grayscale_img_fn, cfg):
        rclpy.init(args=rclpy_args)
        self._node = rclpy.create_node("aruco_publisher")        
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self._node)
        self._broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self._node)
        self._publisher = self._node.create_publisher(ARMarkers, "/aruco/"+cfg['camera_frame'], 10)
        
        self._clock_q = Queue()
        self._node.create_subscription(Clock, "/clock", self.clock_cb, 10)
        
        self._get_grayscale_img_fn = get_grayscale_img_fn
        self._cfg = cfg
        
    def run(self):
        aruco_thread_ = Thread(target=self.aruco_loop)
        aruco_thread_.start()        
        rclpy.spin(self._node)
        
        self._node.destroy_node()    
        rclpy.shutdown()
        aruco_thread_.join()
        
    def clock_cb(self, msg):
        if self._clock_q.qsize() is 0:
            self._clock_q.put([ msg.clock, monotonic() ])
            
    def aruco_loop(self):
        while rclpy.ok():
            img_, img_age_ns_, img_monotonic_stamp_ = self._get_grayscale_img_fn()
            if img_ is not None and self._clock_q.qsize() is not 0:
                try:
                    [ clock_, clock_monotonic_stamp_ ] = self._clock_q.get()
                    img_age_ns_ += int((clock_monotonic_stamp_ - img_monotonic_stamp_) * 1e9)
                    clock_time_ = rclpy.time.Time(seconds=clock_.sec, nanoseconds=clock_.nanosec)
                    clock_time_shifted_msg_ = (clock_time_ - rclpy.duration.Duration(nanoseconds=img_age_ns_)).to_msg()
                    wc_tf_ = self._tf_buffer.lookup_transform(target_frame=self._cfg['parent_frame'], source_frame=self._cfg['camera_frame'], time=clock_time_shifted_msg_)
                except Exception as stamped_lookup_exception:
                    print(stamped_lookup_exception)
                    try:
                        wc_tf_ = self._tf_buffer.lookup_transform(target_frame=self._cfg['parent_frame'], source_frame=self._cfg['camera_frame'], time=Time()) 
                    except Exception as unstamped_lookup_exception:
                        print(unstamped_lookup_exception)
                        continue
                
                markers_msg_ = ARMarkers(header=Header(stamp=wc_tf_.header.stamp, frame_id=self._cfg['parent_frame']))
                if self._cfg['image_scaling'] != 1.0: img_ = cv2.resize(img_, (0,0), fx=self._cfg['image_scaling'], fy=self._cfg['image_scaling'])
                markers_ = get_aruco_markers(img_, self._cfg)
                if len(markers_) is not 0:      
                    for marker in markers_:
                        tf_ = TransformStamped(header=Header(stamp=wc_tf_.header.stamp, frame_id=self._cfg['camera_frame']), child_frame_id=marker.pose.header.frame_id)
                        tf_.transform.translation.x = marker.pose.pose.position.x
                        tf_.transform.translation.y = marker.pose.pose.position.y
                        tf_.transform.translation.z = marker.pose.pose.position.z
                        tf_.transform.rotation.x = marker.pose.pose.orientation.x
                        tf_.transform.rotation.y = marker.pose.pose.orientation.y
                        tf_.transform.rotation.z = marker.pose.pose.orientation.z
                        tf_.transform.rotation.w = marker.pose.pose.orientation.w
                        self._broadcaster.sendTransform(tf_)
                        
                        ttf_ = multiply_transforms(wc_tf_.transform, tf_.transform)
                        marker.pose.pose.position.x = ttf_.translation.x
                        marker.pose.pose.position.y = ttf_.translation.y
                        marker.pose.pose.position.z = ttf_.translation.z
                        marker.pose.pose.orientation.x = ttf_.rotation.x
                        marker.pose.pose.orientation.y = ttf_.rotation.y
                        marker.pose.pose.orientation.z = ttf_.rotation.z
                        marker.pose.pose.orientation.w = ttf_.rotation.w
                        markers_msg_.markers.append(marker)
                    
                self._publisher.publish(markers_msg_)
                
            else:
                print([ img_ is not None, self._clock_q.qsize() ])
                sleep(0.5)
    

    
def generate_k(cx, cy, fx, fy):
    return np.array([[ fx, 0, cx ], \
                     [ 0, fy, cy ], \
                     [ 0, 0, 1 ]])
                     
def load_json(fp):
    if isfile(fp):
        with open(fp, 'r') as f:
            dict_ = json.load(f)
    else: dict_ = {}
            
    return dict_
    
def load_aruco_dict(s):
    # https://docs.opencv.org/master/dc/df7/dictionary_8hpp.html
    if   s == '4X4_50':         return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    elif s == '4X4_100':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    elif s == '4X4_250':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    elif s == '4X4_1000':       return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
    elif s == '5X5_50':         return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
    elif s == '5X5_100':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
    elif s == '5X5_250':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
    elif s == '5X5_1000':       return cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    elif s == '6X6_50':         return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    elif s == '6X6_100':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)
    elif s == '6X6_250':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    elif s == '6X6_1000':       return cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
    elif s == '7X7_50':         return cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
    elif s == '7X7_100':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_100)
    elif s == '7X7_250':        return cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_250)
    elif s == '7X7_1000':       return cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)
    elif s == 'ARUCO_ORIGINAL': return cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    elif s == 'APRILTAG_16h5':  return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
    elif s == 'APRILTAG_25h9':  return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_25h9)
    elif s == 'APRILTAG_36h10': return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h10)
    elif s == 'APRILTAG_36h11': return cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    else: return {}
                     
def load_aruco_params(aruco_params_dict):
    out_ = cv2.aruco.DetectorParameters_create()
    
    for item in aruco_params_dict.items():
        setattr(out_, item[0], item[1])
    
    return out_
    
def load_config(fp, K, d):
    config_ = load_json(fp)
    
    config_['K_'] = K * config_['image_scaling']
    config_['d_'] = d
    
    config_['aruco_dict_'] = load_aruco_dict(config_['aruco_dict'])
    config_['aruco_params_'] = load_aruco_params(config_['aruco_params'])
    
    return config_
    
    
def zed(rclpy_args=None):    
    import pyzed.sl as sl
    
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    if len(sys.argv) > 2:
        address_ = sys.argv[2]
        colon_idx_ = address_.find(':')
        init_params.set_from_stream(address_[:colon_idx_], int(address_[colon_idx_+1:]))

    zed = sl.Camera()
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS: exit(1)
    
    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()
    def get_grayscale_img():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img_grayscale_ = cv2.cvtColor(image.get_data()[:,:,:3], cv2.COLOR_BGR2GRAY)
            age_ns_ = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds() - zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            return img_grayscale_, age_ns_, monotonic()
        else: return None, 0, 0
        
    left_cam_calibration_ = zed.get_camera_information().calibration_parameters.left_cam
    K_ = generate_k(left_cam_calibration_.cx, left_cam_calibration_.cy, left_cam_calibration_.fx, left_cam_calibration_.fy)
    d_ = np.asarray(left_cam_calibration_.disto)
    
    config_ = load_config(sys.argv[1], K_, d_)
    ArucoBroadcaster(rclpy_args, get_grayscale_img, config_).run()
    
    zed.close()
    
def grpc(rclpy_args=None):
    import grpc, ar_tools.Image_pb2_grpc, ar_tools.Common_pb2, ar_tools.Common_pb2_grpc
    from struct import unpack
    from zlib import crc32
    
    global grpc_thread_continue
    grpc_thread_continue = True
    grayscale_img_queue = Queue()
    grayscale_img_data_out_ = None    
    
    image_channel_ = grpc.insecure_channel(sys.argv[2])
    image_client_ = ar_tools.Image_pb2_grpc.ImageServerStub(image_channel_)
    common_channel_ = grpc.insecure_channel(sys.argv[3])
    common_client_ = ar_tools.Common_pb2_grpc.CommonServerStub(common_channel_)
    empty_ = ar_tools.Common_pb2.Empty()   
    
    def grpc_grayscale_img_thread(image_client, q):
        global grpc_thread_continue
        
        stream_ = image_client.stream(empty_)
                         
        while grpc_thread_continue:
            data_ = next(stream_, None).data
            if data_ is not None:
                crc_from_header_ = unpack('I', data_[20:24])[0]                
                image_data_ = data_[28:]
                if crc_from_header_ == crc32(image_data_):
                    image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
                    grayscale_img_data_ = cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)
                    grayscale_img_ts_ = unpack('Q', data_[4:12])[0]
                    if q.qsize() is 0: q.put([ grayscale_img_data_, grayscale_img_ts_ ])
        
    def get_grayscale_img():
        if grayscale_img_queue.qsize() is not 0:
            [ grayscale_img_data_out_, grayscale_img_ts_ ] = grayscale_img_queue.get()
            age_monotonic_before_ = monotonic()
            grayscale_img_age_ns_ = common_client_.get_timestamp(empty_).stamp - grayscale_img_ts_
            age_monotonic_ = (monotonic() + age_monotonic_before_) / 2        
            return grayscale_img_data_out_, grayscale_img_age_ns_, age_monotonic_
        else: return None, 0, 0
    
    metadata_ = image_client_.get_metadata(ar_tools.Common_pb2.Empty())
    K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
    d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
    
    grpc_thread_ = Thread(target=grpc_grayscale_img_thread, args=( image_client_, grayscale_img_queue ))
    grpc_thread_.start()
    
    config_ = load_config(sys.argv[1], K_, d_)
    ArucoBroadcaster(rclpy_args, get_grayscale_img, config_).run()
    
    grpc_thread_continue = False
    grpc_thread_.join()

