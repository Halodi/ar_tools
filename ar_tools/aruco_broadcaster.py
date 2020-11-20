import sys, cv2
import numpy as np
from time import monotonic, sleep

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
    
    address_ = sys.argv[2]
    colon_idx_ = address_.find(':')
    if colon_idx_ != -1:
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
    from multiprocessing import Process, Pipe
    import grpc, ar_tools.Image_pb2_grpc, ar_tools.Common_pb2, ar_tools.Common_pb2_grpc
    from struct import unpack
    from zlib import crc32
    
    def wait_for_channel_ready(channel):
        ready_ = [ False ]
        
        def cc_cb(cc):
            if cc is grpc.ChannelConnectivity.READY:
                ready_[0] = True
            
        channel.subscribe(cc_cb, True)
        while not ready_[0]: sleep(0.1)
        channel.unsubscribe(cc_cb)
    
    grpc_timeout_ = 1.0
    common_channel_ = grpc.insecure_channel(sys.argv[3])
    wait_for_channel_ready(common_channel_)
    common_client_ = ar_tools.Common_pb2_grpc.CommonServerStub(common_channel_)
    empty_ = ar_tools.Common_pb2.Empty()    
    
    pipeA_, pipeB_ = Pipe()
    
    def get_grayscale_img():
        pipeA_.send(True)
        
        if pipeA_.poll(grpc_timeout_):
            [ grayscale_img_data_out_, grayscale_img_ts_ ] = pipeA_.recv()            
            try:
                monotonic_before_ = monotonic()
                server_ts_ = common_client_.get_timestamp(empty_, timeout=grpc_timeout_).stamp
                age_ns_ = server_ts_ - grayscale_img_ts_
                monotonic_mean_ = (monotonic() + monotonic_before_) / 2
                return grayscale_img_data_out_, monotonic_mean_ - (age_ns_ / 1e9)
            except: print("Unable to get image server timestamp")
        else: print("Unable to get data from gRPC process")
        
        return None, 0
    
    def grpc_grayscale_img_thread(image_server_address, pipe):            
        image_channel_ = grpc.insecure_channel(image_server_address)
        wait_for_channel_ready(image_channel_)
        image_client_ = ar_tools.Image_pb2_grpc.ImageServerStub(image_channel_)
        
        metadata_ = image_client_.get_metadata(empty_)
        K_ = generate_k(metadata_.image_center.x, metadata_.image_center.y, metadata_.focal_length_px.x, metadata_.focal_length_px.y)
        d_ = np.array([ metadata_.k[0], metadata_.k[1], metadata_.p[0], metadata_.p[1] ] + metadata_.k[2:])
        pipe.send([ K_, d_ ])

        stream_ = image_client_.stream(empty_)
        data_wanted_ = False
        start_time_ = monotonic()
        while True:
            try:
                data_ = next(stream_, None).data
            except:
                print("Hit end of gRPC stream")
                break
                
            if pipe.poll():
                pipe_data_ = pipe.recv()
                if pipe_data_ is True:
                    data_wanted_ = True
                else:
                    print("gRPC stream client got exit request over pipe")
                    break

            if data_wanted_ and data_ is not None:
                if len(data) < 28: continue
                crc_from_header_ = unpack('I', data_[20:24])[0] 
                image_data_ = data_[28:]
                if crc_from_header_ == crc32(image_data_):
                    try:
                        image_data_np_ = np.asarray(unpack('%dB'%len(image_data_), image_data_)).astype(np.uint8)
                        grayscale_img_data_ = cv2.imdecode(image_data_np_, cv2.IMREAD_GRAYSCALE)
                        grayscale_img_ts_ = unpack('Q', data_[4:12])[0]
                        pipe.send([ grayscale_img_data_, grayscale_img_ts_ ])
                        data_wanted_ = False
                    except: continue
    
    grpc_stream_recv_ = Process(target=grpc_grayscale_img_thread, args=( sys.argv[2], pipeB_ ))
    grpc_stream_recv_.start()
    
    [ K_, d_ ] = pipeA_.recv()
    config_ = load_config(sys.argv[1], K_, d_)
    ArucoBroadcaster(rclpy_args, get_grayscale_img, config_).run()
    
    pipeA_.send(False)
    grpc_stream_recv_.join()

