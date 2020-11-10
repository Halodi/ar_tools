import sys
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy, tf2_ros
from ar_tools.transforms_math import multiply_transforms
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from halodi_msgs.msg import ARMarker, ARMarkers
from builtin_interfaces.msg import Time
from std_msgs.msg import Header



# https://docs.opencv.org/master/d1/dcd/structcv_1_1aruco_1_1DetectorParameters.html
aruco_parameters = cv2.aruco.DetectorParameters_create()
# https://docs.opencv.org/master/dc/df7/dictionary_8hpp.html
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)

marker_length_dict = { \
    0: 0.15 \
}



marker_quat_flip = Rotation.from_quat([1,0,0,0])

def get_aruco_markers(image_grayscale, K, d):
    corners, ids, _rejectedImgPoints = cv2.aruco.detectMarkers(image_grayscale, aruco_dict, parameters=aruco_parameters)
    msgs_ = []
    for i in range(len(corners)):
        for j in range(len(ids[i])):
            id_ = ids[i][j]
            if id_ not in marker_length_dict: continue
            
            msg_ = ARMarker(data="")
            msg_.pose.header.frame_id = str(id_)
            for corner in corners[i][j]:
                msg_.points.append(Point(x=float(corner[0]), y=float(corner[1]), z=0.0))
                
            rvecs, tvecs = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length_dict[id_], K, d)
            
            tvecs_flat_ = tvecs.ravel()
            msg_.pose.pose.position.x =  tvecs_flat_[2]
            msg_.pose.pose.position.y = -tvecs_flat_[0]
            msg_.pose.pose.position.z = -tvecs_flat_[1]
             
            quat_ = (marker_quat_flip * Rotation.from_rotvec(rvecs.ravel())).as_quat()
            msg_.pose.pose.orientation.x = -quat_[2]
            msg_.pose.pose.orientation.y = -quat_[0]
            msg_.pose.pose.orientation.z =  quat_[1]
            msg_.pose.pose.orientation.w =  quat_[3]
            msgs_.append(msg_)
        
    return msgs_
    
def run(rclpy_args, get_grayscale_img_fn, K, d, parent_frame_id="map", frame_id="camera"):
    rclpy.init(args=rclpy_args)
    node_ = rclpy.create_node("aruco_publisher")
    broadcaster_ = tf2_ros.transform_broadcaster.TransformBroadcaster(node_)
    tf_buffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tf_buffer_, node_, spin_thread=True)
    publisher_ = node_.create_publisher(ARMarkers, "/aruco/"+frame_id, 10)
    
    while rclpy.ok():
        img_ = get_grayscale_img_fn()        
        if img_ is not None:            
            markers_ = get_aruco_markers(img_, K, d)
            ts_ = node_.get_clock().now().to_msg()
            markers_msg_ = ARMarkers(header = Header(stamp=ts_, frame_id=parent_frame_id))
            wc_tf_ = tf_buffer_.lookup_transform(target_frame=parent_frame_id, source_frame=frame_id, time=Time())
            
            for marker in markers_:
                tf_ = TransformStamped(header=Header(stamp=ts_, frame_id=frame_id), child_frame_id=marker.pose.header.frame_id)
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
            
            if len(markers_msg_.markers) is not 0:
                publisher_.publish(markers_msg_)
                
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
        init_params.set_from_stream(sender_ip=address_[:colon_idx_], port=int(address_[colon_idx_+1:]))

    zed = sl.Camera()
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS: exit(1)
    
    runtime_parameters = sl.RuntimeParameters()
    image = sl.Mat()
    def get_grayscale_img():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            return cv2.cvtColor(image.get_data()[:,:,:3], cv2.COLOR_BGR2GRAY)
        else: return None
        
    left_cam_calibration_ = zed.get_camera_information().calibration_parameters.left_cam
    K_ = generate_k(left_cam_calibration_.cx, left_cam_calibration_.cy, left_cam_calibration_.fx, left_cam_calibration_.fy)
    d_ = np.asarray(left_cam_calibration_.disto)
    
    run(rclpy_args, get_grayscale_img, K_, d_, sys.argv[1], sys.argv[2])
    
    zed.close()
    
def grpc(rclpy_args=None):
    import grpc, ar_tools.Image_pb2_grpc, ar_tools.Common_pb2
    from struct import unpack
    from zlib import crc32
    
    channel_ = grpc.insecure_channel(sys.argv[3])
    client_ = ar_tools.Image_pb2_grpc.ImageServerStub(channel_)
    stream_ = client_.stream(ar_tools.Common_pb2.Empty())
    
    def get_grayscale_img():
        data_ = next(stream_).data
        crc_from_header_ = unpack('I', data_[20:24])[0]
        image_data_ = data_[28:]
        if crc_from_header_ == crc32(image_data_):
            image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
            return cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)            
        else: return None
    
    metadata_ = client_.get_metadata(ar_tools.Common_pb2.Empty())
    K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
    d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
    
    run(rclpy_args, get_grayscale_img, K_, d_, sys.argv[1], sys.argv[2])
