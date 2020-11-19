import rclpy, tf2_ros
from rclpy.qos import *
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from halodi_msgs.srv import GetStampedTF

from threading import Lock
from time import monotonic



class STF_Server(rclpy.node.Node):
    def __init__(self):
        super().__init__('stf_server')
        
        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer, self,
            qos = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT),
            static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT))
            
        self._lock = Lock()
        self._latest_clock = [ 0, 0 ]    
        self.create_subscription(Clock, "/clock", self.clock_cb, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._srv = self.create_service(GetStampedTF, 'get_stamped_tf', self.get_stf_srv)
        
    def clock_cb(self, msg):
        with self._lock:
            self._latest_clock[0] = msg.clock.sec
            self._latest_clock[1] = msg.clock.nanosec
            
    def get_stf_srv(self, request, response):
        with self._lock:            
            try:
                age_ns_ = int((monotonic() - request.monotonic_stamp) * 1e9)
                ts_ = rclpy.time.Time(seconds=self._latest_clock[0], nanoseconds=self._latest_clock[1]) - Duration(nanoseconds=age_ns_)
                response.stf = self._tf_buffer.lookup_transform(target_frame=request.parent_frame, source_frame=request.child_frame, time=ts_)
                response.ok = True
            except Exception as e:
                print(e)    
                response.ok = False
                
        return response
        
        

def main(args=None):
    rclpy.init(args=args)
    stfs_ = STF_Server()
    rclpy.spin(stfs_)
    rclpy.shutdown()
