from geometry_msgs.msg import Transform
import numpy as np
from scipy.spatial.transform import Rotation

def transform_to_matrix(tf):
    out_ = np.empty([4,4])
    out_[3,:] = [ 0, 0, 0, 1 ]
    
    out_[0,3] = tf.translation.x
    out_[1,3] = tf.translation.y
    out_[2,3] = tf.translation.z
    
    out_[:3,:3] = Rotation.from_quat([ tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w ]).as_matrix()
    
    return out_
    
def pose_to_matrix(pose):
    out_ = np.empty([4,4])
    out_[3,:] = [ 0, 0, 0, 1 ]
    
    out_[0,3] = pose.position.x
    out_[1,3] = pose.position.y
    out_[2,3] = pose.position.z
    
    out_[:3,:3] = Rotation.from_quat([ pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w ]).as_matrix()
    
    return out_
    
def matrix_to_transform(m):
    out_ = Transform()
    
    out_.translation.x = m[0,3]
    out_.translation.y = m[1,3]
    out_.translation.z = m[2,3]
    
    quat_ = Rotation.from_matrix(m[:3,:3]).as_quat()
    out_.rotation.x = quat_[0]
    out_.rotation.y = quat_[1]
    out_.rotation.z = quat_[2]
    out_.rotation.w = quat_[3]
    
    return out_
    
def multiply_transforms(tfA, tfB):
    return matrix_to_transform(np.matmul(transform_to_matrix(tfA), transform_to_matrix(tfB)))

def invert_transform_matrix(m):
    out_ = np.empty([4,4])
    out_[3,:] = [ 0, 0, 0, 1 ]
    out_[:3,:3] = m[:3,:3].T
    out_[:3,3] = -np.matmul(out_[:3,:3], m[:3,3])

    return out_
