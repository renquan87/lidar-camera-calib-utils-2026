import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Camera Info Publisher has been started.')

    def timer_callback(self):
        msg = CameraInfo()
        # Fill in the CameraInfo message fields here
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = 2048
        msg.width = 3072
        msg.distortion_model = 'plumb_bob'
        msg.d = [-0.116015, 0.156237, 0.000245, -0.000297, 0.000000]
        msg.k = [3405.47215,    0.     , 1512.51235,
            0.     , 3405.51992, 1004.18006,
            0.     ,    0.     ,    1.     ]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [1.0, 0.0, 320.0, 0.0, 0.0, 1.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing camera info')

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
