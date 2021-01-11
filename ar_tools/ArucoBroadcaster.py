import json, cv2
import numpy as np
from scipy.spatial.transform import Rotation
from os.path import isfile
from os import getpid
from ar_tools.transforms_math import multiply_transforms

import rclpy, tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Point
from std_msgs.msg import Header
from halodi_msgs.msg import ARMarker, ARMarkers
from halodi_msgs.srv import GetStampedTF



class ArucoBroadcaster:
    def __init__(self, rclpy_args, get_grayscale_img_fn, cfg):
        self._get_grayscale_img_fn = get_grayscale_img_fn
        self._cfg = cfg
        self._marker_rot_adj = Rotation.from_rotvec([ 0, np.pi, 0 ])
    
        rclpy.init(args=rclpy_args)
        self._node = rclpy.create_node("aruco_publisher_" + str(getpid()))
        
        self._stf_client = self._node.create_client(GetStampedTF, 'get_stamped_tf')
        self._stf_req = GetStampedTF.Request(parent_frame=self._cfg['parent_frame'], child_frame=self._cfg['camera_frame'])
              
        self._broadcaster = tf2_ros.TransformBroadcaster(self._node)
        self._publisher = self._node.create_publisher(ARMarkers, "aruco/"+self._cfg['camera_frame'], 10)
        self._publisher_tf = self._node.create_publisher(ARMarkers, "aruco/"+self._cfg['parent_frame'], 10) if self._cfg['parent_frame'] != self._cfg['camera_frame'] else None
        
    def run(self):
        while rclpy.ok():
            [ img_, img_monotonic_stamp_ ] = self._get_grayscale_img_fn()
            
            if img_ is not None:
                try:
                    self._stf_req.monotonic_stamp = img_monotonic_stamp_ if self._cfg['apply_timestamp_age'] else -1.0
                    future_ = self._stf_client.call_async(self._stf_req)
                    rclpy.spin_until_future_complete(self._node, future_)
                
                    res_ = future_.result()
                    if res_.ok:
                        self.find_and_publish_aruco_markers(img_, res_.stf)
                    else:
                        self._node.get_logger().error("Received an invalid stamped transform")
                        continue
                except:
                    self._node.get_logger().error("Unable to receive a stamped transform")
                    continue
            else: self._node.get_logger().error("Unable to get image data")
                
        self._node.destroy_node()
        rclpy.shutdown()
        
    def find_and_publish_aruco_markers(self, image_grayscale, world_to_camera_stf):
        if self._cfg['image_scaling'] != 1.0:
            image_grayscale = cv2.resize(image_grayscale, (0,0), fx=self._cfg['image_scaling'], fy=self._cfg['image_scaling'])
            
        markers_msg_ = ARMarkers(header=Header(stamp=world_to_camera_stf.header.stamp, frame_id=self._cfg['camera_frame']), markers=self.get_aruco_markers(image_grayscale))
        self._publisher.publish(markers_msg_)

        tfs_ = []
        for marker in markers_msg_.markers:
            tf_ = TransformStamped(header=Header(stamp=world_to_camera_stf.header.stamp, frame_id=self._cfg['camera_frame']), child_frame_id=marker.pose.header.frame_id)
            tf_.transform.translation.x = marker.pose.pose.position.x
            tf_.transform.translation.y = marker.pose.pose.position.y
            tf_.transform.translation.z = marker.pose.pose.position.z
            tf_.transform.rotation.x = marker.pose.pose.orientation.x
            tf_.transform.rotation.y = marker.pose.pose.orientation.y
            tf_.transform.rotation.z = marker.pose.pose.orientation.z
            tf_.transform.rotation.w = marker.pose.pose.orientation.w
            tfs_.append(tf_)
        if len(tfs_) != 0: self._broadcaster.sendTransform(tfs_)
        
        if self._publisher_tf is not None:
            for i in range(len(tfs_)):
                ttf_ = multiply_transforms(world_to_camera_stf.transform, tfs_[i].transform)
                markers_msg_.markers[i].pose.pose.position.x = ttf_.translation.x
                markers_msg_.markers[i].pose.pose.position.y = ttf_.translation.y
                markers_msg_.markers[i].pose.pose.position.z = ttf_.translation.z
                markers_msg_.markers[i].pose.pose.orientation.x = ttf_.rotation.x
                markers_msg_.markers[i].pose.pose.orientation.y = ttf_.rotation.y
                markers_msg_.markers[i].pose.pose.orientation.z = ttf_.rotation.z
                markers_msg_.markers[i].pose.pose.orientation.w = ttf_.rotation.w
            markers_msg_.header.frame_id = self._cfg['parent_frame']
            self._publisher_tf.publish(markers_msg_)
                
    def get_aruco_markers(self, image_grayscale):
        corners, ids, _rejectedImgPoints = cv2.aruco.detectMarkers(image_grayscale, self._cfg['aruco_dict_'], parameters=self._cfg['aruco_params_'])
        msgs_ = []
        for i in range(len(corners)):
            id_ = str(ids[i][0])
            if id_ not in self._cfg['marker_sizes'].keys(): continue
            
            msg_ = ARMarker(data="")
            msg_.pose.header.frame_id = id_
            for corner in corners[i][0]:
                msg_.points.append(Point(x=float(corner[0]), y=float(corner[1]), z=0.0))
                
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], self._cfg['marker_sizes'][id_], cameraMatrix=self._cfg['K_'], distCoeffs=self._cfg['d_'])
            
            tvecs_flat_ = tvecs.ravel()
            msg_.pose.pose.position.x =  tvecs_flat_[2]
            msg_.pose.pose.position.y = -tvecs_flat_[0]
            msg_.pose.pose.position.z = -tvecs_flat_[1]
            
            rvecs_flat_ = rvecs.ravel()
            rot_ = Rotation.from_rotvec([ rvecs_flat_[2], -rvecs_flat_[0], -rvecs_flat_[1] ]) * self._marker_rot_adj
            quat_ = rot_.as_quat()
            msg_.pose.pose.orientation.x = quat_[0]
            msg_.pose.pose.orientation.y = quat_[1]
            msg_.pose.pose.orientation.z = quat_[2]
            msg_.pose.pose.orientation.w = quat_[3]
            msgs_.append(msg_)

        return msgs_
        
        

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
    else: return None
    
def load_aruco_params(aruco_params_dict):
    out_ = cv2.aruco.DetectorParameters_create()
    
    for item in aruco_params_dict.items():
        setattr(out_, item[0], item[1])
    
    return out_
    
def load_config(fp, K, d):
    if isfile(fp):
        with open(fp, 'r') as f:
            config_ = json.load(f)            
            config_['K_'] = K * config_['image_scaling']
            config_['d_'] = d            
            config_['aruco_dict_'] = load_aruco_dict(config_['aruco_dict'])
            config_['aruco_params_'] = load_aruco_params(config_['aruco_params'])
        return config_
    else: return None
