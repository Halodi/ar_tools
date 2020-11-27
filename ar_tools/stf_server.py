import sys
from threading import Lock
from time import perf_counter
from ar_tools.throttle import Throttle

import rclpy, tf2_ros
from rclpy.qos import *
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from halodi_msgs.srv import GetStampedTF



class STF_Server(rclpy.node.Node):
    def __init__(self):
        super().__init__('stf_server')
            
        self._lock = Lock()
        self._latest_clock = [ 0, 0 ]
        self._tf_buffer_core = tf2_ros.BufferCore()
        self._sensor_delay_durations = {}
          
        self.create_subscription(Clock, "/clock", self.clock_cb, QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(TFMessage, "/tf", self.tf_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(TFMessage, "/tf_static", self.tf_static_cb,
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST))
            
        self._srv = self.create_service(GetStampedTF, 'get_stamped_tf', self.get_stf_srv)
        
    def clock_cb(self, msg):
        with self._lock:
            self._latest_clock[0] = int(msg.clock.sec * 1e9) + msg.clock.nanosec
            self._latest_clock[1] = perf_counter()
            
    def tf_cb(self, msg):
        who = 'default_authority'
        
        for tf in msg.transforms:
            self._tf_buffer_core.set_transform(tf, who)
            
    def tf_static_cb(self, msg):
        with self._lock:
            who = 'default_authority'
                    
            for tf in msg.transforms:
                if tf.header.stamp.sec != 0 or tf.header.stamp.nanosec != 0:
                    ns_ = int(tf.header.stamp.sec * 1e9) + tf.header.stamp.nanosec
                    self._sensor_delay_durations[tf.child_frame_id] = ns_
                    
                self._tf_buffer_core.set_transform_static(tf, who)
            
    def get_stf_srv(self, request, response):
        with self._lock:            
            try:
                age_ns_ = int((self._latest_clock[1] - request.monotonic_stamp) * 1e9)
                sensor_delay_ = self._sensor_delay_durations.get(request.child_frame)
                if sensor_delay_ is not None: age_ns_ += sensor_delay_

                ts_ = rclpy.time.Time(nanoseconds=self._latest_clock[0] - age_ns_)
                response.stf = self._tf_buffer_core.lookup_transform_core(request.parent_frame, request.child_frame, ts_)
                response.ok = True
            except Exception as e:
                self.get_logger().error(str(e))    
                response.ok = False
                
        return response
        
        

def main(args=None):
    rclpy.init(args=args)
    
    stfs_ = STF_Server()
    
    rclpy.spin(stfs_)
    throttle_ = Throttle(int(sys.argv[1]))
    while rclpy.ok():
        rclpy.spin_once(stfs_)
        throttle_.wait()
    
    stfs_.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
