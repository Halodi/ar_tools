import sys, cv2, json
import numpy as np
from scipy.spatial.transform import Rotation
from time import sleep

import rclpy, tf2_ros
from ar_tools.transforms_math import multiply_transforms
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from halodi_msgs.msg import ARMarker, ARMarkers
from builtin_interfaces.msg import Time
from std_msgs.msg import Header

from threading import Thread, Lock, Condition
from rosgraph_msgs.msg import Clock



# https://docs.opencv.org/master/dc/df7/dictionary_8hpp.html
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

marker_length_dict = { \
    0: 0.15 \
}

vis = False
if vis:
    cv2.namedWindow("aruco")
    cv2.startWindowThread()
    
tf_verbose = True



marker_rot_adj = Rotation.from_rotvec([0, np.pi, 0])

def get_aruco_markers(image_grayscale, aruco_parameters, K, d):
    corners, ids, _rejectedImgPoints = cv2.aruco.detectMarkers(image_grayscale, aruco_dict, parameters=aruco_parameters)
    msgs_ = []
    for i in range(len(corners)):
        id_ = ids[i][0]
        if id_ not in marker_length_dict: continue
        
        msg_ = ARMarker(data="")
        msg_.pose.header.frame_id = str(id_)
        for corner in corners[i][0]:
            msg_.points.append(Point(x=float(corner[0]), y=float(corner[1]), z=0.0))
            
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length_dict[id_], cameraMatrix=K, distCoeffs=d)
        
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
            
    if vis:
        cv2.imshow("aruco", image_grayscale)
        
    return msgs_
    
def run(rclpy_args, get_grayscale_img_fn, K, d, parent_frame_id, frame_id, fxy, aruco_parameters):
    rclpy.init(args=rclpy_args)
    global node_
    node_ = rclpy.create_node("aruco_publishoer")
    broadcaster_ = tf2_ros.transform_broadcaster.TransformBroadcaster(node_)
    tf_buffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tf_buffer_, node_)
    publisher_ = node_.create_publisher(ARMarkers, "/aruco/"+frame_id, 10)
    
    global clock_, want_clock_, lock_
    clock_ = Time()
    want_clock_ = False
    lock_ = Condition()
    
    def spin_thread():    
        def clock_callback(msg):
            global lock_
            with lock_:
                global clock_, want_clock_
                clock_.sec = msg.clock.sec
                clock_.nanosec = msg.clock.nanosec
                want_clock_ = False
                lock_.notify_all()
                
        global node_
        clock_subscriber_ = node_.create_subscription(Clock, "/clock", clock_callback, 10)
        rclpy.spin(node_)
        
    spin_thread_ = Thread(target=spin_thread)
    spin_thread_.start()
    
    K = K * fxy
    
    while rclpy.ok():
        img_, img_age_ns_ = get_grayscale_img_fn()
        if img_ is not None:
            with lock_:
                want_clock_ = True
                while want_clock_: lock_.wait()
                clock_s_float_ = clock_.sec + clock_.nanosec / 1e9
                timestamp_s_float_ = clock_s_float_ - (img_age_ns_ / 1e9)
                timestamp_s_ = int(clock_s_float_)
                timestamp_ns_ = abs(int((timestamp_s_float_ - timestamp_s_) * 1e9))
                timestamp_ = Time(sec=timestamp_s_, nanosec=timestamp_ns_)      
                
            try:
                wc_tf_ = tf_buffer_.lookup_transform(target_frame=parent_frame_id, source_frame=frame_id, time=timestamp_)                
            except Exception as stamped_lookup_exception:
                if tf_verbose: print(stamped_lookup_exception)
                try:
                    wc_tf_ = tf_buffer_.lookup_transform(target_frame=parent_frame_id, source_frame=frame_id, time=Time()) 
                except Exception as unstamped_lookup_exception:
                    if tf_verbose: print(unstamped_lookup_exception)
                    continue
            
            markers_msg_ = ARMarkers(header=Header(stamp=wc_tf_.header.stamp, frame_id=parent_frame_id))
            if fxy != 1.0: img_ = cv2.resize(img_, (0,0), fx=fxy, fy=fxy)
            markers_ = get_aruco_markers(img_, aruco_parameters, K, d)
            if len(markers_) is not 0:      
                for marker in markers_:
                    tf_ = TransformStamped(header=Header(stamp=wc_tf_.header.stamp, frame_id=frame_id), child_frame_id=marker.pose.header.frame_id)
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

    node_.destroy_node()    
    rclpy.shutdown()
    
def generate_k(cx, cy, fx, fy):
    return np.array([[ fx, 0, cx ], \
                     [ 0, fy, cy ], \
                     [ 0, 0, 1 ]])
    
    
    
def zed(rclpy_args=None):    
    import pyzed.sl as sl
    
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.camera_fps = 30
    if len(sys.argv) > 3:
        address_ = sys.argv[3]
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
            age_ = 0
            return cv2.cvtColor(image.get_data()[:,:,:3], cv2.COLOR_BGR2GRAY), age_
        else: return None, 0
        
    left_cam_calibration_ = zed.get_camera_information().calibration_parameters.left_cam
    K_ = generate_k(left_cam_calibration_.cx, left_cam_calibration_.cy, left_cam_calibration_.fx, left_cam_calibration_.fy)
    d_ = np.asarray(left_cam_calibration_.disto)
    
    run(rclpy_args, get_grayscale_img, K_, d_, sys.argv[1], sys.argv[2], \
        sys.argv[4] if len(sys.argv) > 4 else 1.0, \
        cv2.aruco.DetectorParameters_create())
    
    zed.close()
    
def grpc(rclpy_args=None):
    import grpc, ar_tools.Image_pb2_grpc, ar_tools.Common_pb2, ar_tools.Common_pb2_grpc
    from struct import unpack
    from zlib import crc32
    
    global lock_
    lock_ = Condition()
    global continue_grpc_thread_
    continue_grpc_thread_ = True
    global grayscale_img_data_, grayscale_img_data_out_, grayscale_img_ts_
    grayscale_img_data_ = None
    grayscale_img_data_out_ = None
    grayscale_img_ts_ = 0
    empty_ = ar_tools.Common_pb2.Empty()
    
    image_channel_ = grpc.insecure_channel(sys.argv[3])
    image_client_ = ar_tools.Image_pb2_grpc.ImageServerStub(image_channel_)
    common_channel_ = grpc.insecure_channel(sys.argv[4])
    common_client_ = ar_tools.Common_pb2_grpc.CommonServerStub(common_channel_)    
    
    def grpc_grayscale_img_thread(image_client):
        global grayscale_img_data_, grayscale_img_ts_, continue_grpc_thread_, lock_
        
        stream_ = image_client.stream(ar_tools.Common_pb2.Empty())
                         
        while continue_grpc_thread_:
            data_ = next(stream_, None).data                
            if data_ != None:
                crc_from_header_ = unpack('I', data_[20:24])[0]                
                image_data_ = data_[28:]
                if crc_from_header_ == crc32(image_data_):
                    with lock_:
                        image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
                        grayscale_img_data_ = cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)
                        grayscale_img_ts_ = unpack('Q', data_[4:12])[0]
                        lock_.notify_all()
        
    def get_grayscale_img():
        global grayscale_img_data_, grayscale_img_data_out_, grayscale_img_ts_
        
        with lock_:
            lock_.wait()
            grayscale_img_data_out_ = np.copy(grayscale_img_data_) if type(grayscale_img_data_) is np.ndarray else None        
            grayscale_img_age_ns_ = common_client_.get_timestamp(empty_).stamp - grayscale_img_ts_
        
        return grayscale_img_data_out_, grayscale_img_age_ns_
    
    metadata_ = image_client_.get_metadata(ar_tools.Common_pb2.Empty())
    K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
    d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
    
    grpc_thread_ = Thread(target=grpc_grayscale_img_thread, args=(image_client_,))
    grpc_thread_.start()
 
    run(rclpy_args, get_grayscale_img, K_, d_, sys.argv[1], sys.argv[2], \
        sys.argv[5] if len(sys.argv) > 5 else 1.0, \
        cv2.aruco.DetectorParameters_create())    
    
    continue_grpc_thread_ = False
    grpc_thread_.join()
