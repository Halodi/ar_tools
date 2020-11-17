import sys, json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from threading import Thread, Condition
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
    
def run(rclpy_args, get_grayscale_img_fn, cfg):
    rclpy.init(args=rclpy_args)
    global node
    node = rclpy.create_node("aruco_publisher")
    broadcaster_ = tf2_ros.transform_broadcaster.TransformBroadcaster(node)
    tf_buffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tf_buffer_, node)
    publisher_ = node.create_publisher(ARMarkers, "/aruco/"+cfg['camera_frame'], 10)
    
    global clock, clock_monotonic_stamp, want_clock, clock_lock
    clock = Time()
    clock_monotonic_stamp = 0.0
    want_clock = False
    clock_lock = Condition()
    
    def spin_thread():    
        def clock_callback(msg):
            global clock, clock_monotonic_stamp, want_clock, clock_lock
            
            if not want_clock: return
            
            with clock_lock:
                clock.sec = msg.clock.sec
                clock.nanosec = msg.clock.nanosec
                clock_monotonic_stamp = monotonic()
                want_clock = False
                clock_lock.notify_all()
                
        global node
        clock_subscriber_ = node.create_subscription(Clock, "/clock", clock_callback, 10)
        rclpy.spin(node)
        
    spin_thread_ = Thread(target=spin_thread)
    spin_thread_.start()
    
    while rclpy.ok():
        img_, img_age_ns_, image_age_ns_monotonic_stamp_ = get_grayscale_img_fn()
        if img_ is not None:
            with clock_lock:
                want_clock = True
                while want_clock: clock_lock.wait()
                clock_s_float_ = clock.sec + clock.nanosec / 1e9
                timestamp_s_float_ = clock_s_float_ - (img_age_ns_ / 1e9) - (clock_monotonic_stamp - image_age_ns_monotonic_stamp_)
                timestamp_s_ = int(timestamp_s_float_)                
                timestamp_ns_ = abs(int((timestamp_s_float_ - timestamp_s_) * 1e9))
                timestamp_ = Time(sec=timestamp_s_, nanosec=timestamp_ns_)
                
            try:
                wc_tf_ = tf_buffer_.lookup_transform(target_frame=cfg['parent_frame'], source_frame=cfg['camera_frame'], time=timestamp_)                
            except Exception as stamped_lookup_exception:
                print(stamped_lookup_exception)
                try:
                    wc_tf_ = tf_buffer_.lookup_transform(target_frame=cfg['parent_frame'], source_frame=cfg['camera_frame'], time=Time()) 
                except Exception as unstamped_lookup_exception:
                    print(unstamped_lookup_exception)
                    continue
            
            markers_msg_ = ARMarkers(header=Header(stamp=wc_tf_.header.stamp, frame_id=cfg['parent_frame']))
            if cfg['image_scaling'] != 1.0: img_ = cv2.resize(img_, (0,0), fx=cfg['image_scaling'], fy=cfg['image_scaling'])
            markers_ = get_aruco_markers(img_, cfg)
            if len(markers_) is not 0:      
                for marker in markers_:
                    tf_ = TransformStamped(header=Header(stamp=wc_tf_.header.stamp, frame_id=cfg['camera_frame']), child_frame_id=marker.pose.header.frame_id)
                    tf_.transform.translation.x = marker.pose.pose.position.x
                    tf_.transform.translation.y = marker.pose.pose.position.y
                    tf_.transform.translation.z = marker.pose.pose.position.z
                    tf_.transform.rotation.x = marker.pose.pose.orientation.x
                    tf_.transform.rotation.y = marker.pose.pose.orientation.y
                    tf_.transform.rotation.z = marker.pose.pose.orientation.z
                    tf_.transform.rotation.w = marker.pose.pose.orientation.w
                    broadcaster_.sendTransform(tf_)
                    
                    ttf_ = multiply_transforms(wc_tf_.transform, tf_.transform)
                    marker.pose.pose.position.x = ttf_.translation.x
                    marker.pose.pose.position.y = ttf_.translation.y
                    marker.pose.pose.position.z = ttf_.translation.z
                    marker.pose.pose.orientation.x = ttf_.rotation.x
                    marker.pose.pose.orientation.y = ttf_.rotation.y
                    marker.pose.pose.orientation.z = ttf_.rotation.z
                    marker.pose.pose.orientation.w = ttf_.rotation.w
                    markers_msg_.markers.append(marker)
                
            publisher_.publish(markers_msg_)
        
        else: sleep(0.5)

    node.destroy_node()    
    rclpy.shutdown()
    
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
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS: exit(1)
    
    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()
    def get_grayscale_img():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img_grayscale_ = cv2.cvtColor(image.get_data()[:,:,:3], cv2.COLOR_BGR2GRAY)
            age_ns_ = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT).get_nanoseconds() - zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            monotonic_stamp_ = monotonic()
            return img_grayscale_, age_ns_, monotonic_stamp_
        else: return None, 0, 0.0
        
    left_cam_calibration_ = zed.get_camera_information().calibration_parameters.left_cam
    K_ = generate_k(left_cam_calibration_.cx, left_cam_calibration_.cy, left_cam_calibration_.fx, left_cam_calibration_.fy)
    d_ = np.asarray(left_cam_calibration_.disto)
    
    config_ = load_config(sys.argv[1], K_, d_)
    run(rclpy_args, get_grayscale_img, config_)
    
    zed.close()
    
def grpc(rclpy_args=None):
    import grpc, ar_tools.Image_pb2_grpc, ar_tools.Common_pb2, ar_tools.Common_pb2_grpc
    from struct import unpack
    from zlib import crc32
    
    global grpc_img_lock, grpc_thread_continue
    grpc_img_lock = Condition()
    grpc_thread_continue = True
    global grayscale_img_data, grayscale_img_ts, grayscale_img_data_out_
    grayscale_img_data = None    
    grayscale_img_ts = 0
    grayscale_img_data_out_ = None    
    
    image_channel_ = grpc.insecure_channel(sys.argv[2])
    image_client_ = ar_tools.Image_pb2_grpc.ImageServerStub(image_channel_)
    common_channel_ = grpc.insecure_channel(sys.argv[3])
    common_client_ = ar_tools.Common_pb2_grpc.CommonServerStub(common_channel_)
    empty_ = ar_tools.Common_pb2.Empty()   
    
    def grpc_grayscale_img_thread(image_client):
        global grayscale_img_data, grayscale_img_ts, grpc_thread_continue, grpc_img_lock
        
        stream_ = image_client.stream(empty_)
                         
        while grpc_thread_continue:
            data_ = next(stream_, None).data                
            if data_ != None:
                crc_from_header_ = unpack('I', data_[20:24])[0]                
                image_data_ = data_[28:]
                if crc_from_header_ == crc32(image_data_):
                    with grpc_img_lock:
                        image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
                        grayscale_img_data = cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)
                        grayscale_img_ts = unpack('Q', data_[4:12])[0]
                        grpc_img_lock.notify_all()
        
    def get_grayscale_img():
        global grayscale_img_data, grayscale_img_ts, grayscale_img_data_out_
        
        with grpc_img_lock:
            grpc_img_lock.wait()
            grayscale_img_data_out_ = np.copy(grayscale_img_data) if type(grayscale_img_data) is np.ndarray else None
            before_ = monotonic()
            grayscale_img_age_ns_ = common_client_.get_timestamp(empty_).stamp - grayscale_img_ts
            after_ = monotonic()
            monotonic_stamp_ = (before_ + after_) / 2
        
        return grayscale_img_data_out_, grayscale_img_age_ns_, monotonic_stamp_
    
    metadata_ = image_client_.get_metadata(ar_tools.Common_pb2.Empty())
    K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
    d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
    
    grpc_thread_ = Thread(target=grpc_grayscale_img_thread, args=(image_client_,))
    grpc_thread_.start()
    
    config_ = load_config(sys.argv[1], K_, d_)
    run(rclpy_args, get_grayscale_img, config_)
    
    continue_grpc_thread_ = False
    grpc_thread_.join()

