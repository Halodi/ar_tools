import numpy as np
import pickle
import os
from geometry_msgs.msg import TransformStamped

def save_matrix_with_metadata(partial_path, M, meta):
    np.save(partial_path+"_m.npz", M)
    pickle.dump(meta, open(partial_path+"_meta.pkl", 'wb'), pickle.HIGHEST_PROTOCOL)
    
def load_matrix_with_metadata(partial_path):
    M_ = np.load(partial_path+"_m.npz")
    meta_ = pickle.load(open(partial_path+"_meta.pkl", 'rb'))
    
    return M_, meta_

def save_ar_marker_data(partial_path, ar_marker_data):
    num_entries_ = len(ar_marker_data)
    M_ = np.empty([num_entries_,4,4])
    meta_ = []
    
    for i in range(num_entries_):
        M_[i] = ar_marker_data[i][0]
        meta_.append(ar_marker_data[i][1:])
        
    save_matrix_with_metadata(partial_path, M_, meta_)
    
def load_ar_marker_data(partial_path):
    M_, meta_ = load_matrix_with_metadata(partial_path)
    
    out_ = []
    for i in range(len(meta_)):
        v_ = [ M_[i] ]
        v_.extend(meta_[i])
        out_.append(v_)
        
    return out_
    
def save_transformstamped_array(partial_path, stfs):
    num_entries_ = len(stfs)
    M_ = np.empty([num_entries_,7])
    meta_ = []
    for i in range(num_entries_):
        M_[i] = [ stfs[i].transform.translation.x,
                  stfs[i].transform.translation.y,
                  stfs[i].transform.translation.z,
                  stfs[i].transform.rotation.x,
                  stfs[i].transform.rotation.y,
                  stfs[i].transform.rotation.z,
                  stfs[i].transform.rotation.w ]
        meta_.append([ stfs[i].header.stamp.sec,
                       stfs[i].header.stamp.nanosec,
                       stfs[i].header.frame_id,
                       stfs[i].child_frame_id ])
                           
    save_matrix_with_metadata(partial_path, M_, meta_)
    
def load_transformstamped_array(partial_path):
    M_, meta_ = load_matrix_with_metadata(partial_path)
    
    out_ = []
    for i in range(len(meta_)):
        stf_ = TransformStamped()
        stf_.header.stamp.sec = meta_[i][0]
        stf_.header.stamp.nanosec = meta_[i][1]
        stf_.header.frame_id = meta_[i][2]
        stf_.child_frame_id = meta_[i][3]
        
        stf_.transform.translation.x = M_[i,0]
        stf_.transform.translation.y = M_[i,1]
        stf_.transform.translation.z = M_[i,2]
        stf_.transform.rotation.x = M_[i,3]
        stf_.transform.rotation.y = M_[i,4]
        stf_.transform.rotation.z = M_[i,5]
        stf_.transform.rotation.w = M_[i,6]
        
        out_.append(stf_)
        
    return out_

def save_calib_data(path, ar_marker_data, tf_msgs, tf_static_msgs):
    save_ar_marker_data(os.path.join(path, "ar_marker_data"), ar_marker_data)    
    save_transformstamped_array(os.path.join(path, "tf"), tf_msgs)
    save_transformstamped_array(os.path.join(path, "tf_static"), tf_static_msgs)
    
def load_calib_data(path):
    ar_marker_data_ = load_ar_marker_data(os.path.join(path, "ar_marker_data"))
    tf_msgs_ = load_transformstamped_array(os.path.join(path, "tf"))
    tf_static_msgs_ = load_transformstamped_array(os.path.join(path, "tf_static"))
    
    return ar_marker_data_, tf_msgs_, tf_static_msgs_
