import sys, cv2
import numpy as np

from threading import Thread
from queue import Queue
from time import monotonic

from ar_tools.ArucoBroadcaster import ArucoBroadcaster, load_config
    
def generate_k(cx, cy, fx, fy):
    return np.array([[ fx, 0, cx ], \
                     [ 0, fy, cy ], \
                     [ 0, 0, 1 ]])
    
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
            return img_grayscale_, monotonic() - (age_ns_ / 1e9)
        else: return None, 0
        
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
    
    def seek_latest_in_fifo_queue(q):
        for i in range(1, q.qsize()):
            _ = q.get()
        
        return q.get()
    
    grayscale_img_queue_ = Queue()
    
    def get_grayscale_img():
        try:
            [ grayscale_img_data_out_, grayscale_img_ts_ ] = seek_latest_in_fifo_queue(grayscale_img_queue_)
            monotonic_before_ = monotonic()
            age_ns_ = common_client_.get_timestamp(empty_).stamp - grayscale_img_ts_
            monotonic_mean_ = (monotonic() + monotonic_before_) / 2
            return grayscale_img_data_out_, monotonic_mean_ - (age_ns_ / 1e9)
        except Exception as e:
            print(e)
            return None, 0
    
    def grpc_grayscale_img_thread(image_client, thread_continue, q):        
        stream_ = image_client.stream(empty_)                         
        while thread_continue[0]:
            data_ = next(stream_, None).data
            if data_ is not None:
                crc_from_header_ = unpack('I', data_[20:24])[0] 
                image_data_ = data_[28:]
                if crc_from_header_ == crc32(image_data_):
                    image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
                    grayscale_img_data_ = cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)
                    grayscale_img_ts_ = unpack('Q', data_[4:12])[0]
                    q.put([ grayscale_img_data_, grayscale_img_ts_ ])
                    
    image_channel_ = grpc.insecure_channel(sys.argv[2])
    image_client_ = ar_tools.Image_pb2_grpc.ImageServerStub(image_channel_)
    common_channel_ = grpc.insecure_channel(sys.argv[3])
    common_client_ = ar_tools.Common_pb2_grpc.CommonServerStub(common_channel_)
    empty_ = ar_tools.Common_pb2.Empty()
    
    metadata_ = image_client_.get_metadata(empty_)
    K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
    d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
    
    grpc_thread_continue_ = [ True ]
    grpc_thread_ = Thread(target=grpc_grayscale_img_thread, args=( image_client_, grpc_thread_continue_, grayscale_img_queue_ ))
    grpc_thread_.start()
    
    config_ = load_config(sys.argv[1], K_, d_)
    ArucoBroadcaster(rclpy_args, get_grayscale_img, config_).run()
    
    grpc_thread_continue_[0] = False
    grpc_thread_.join()

