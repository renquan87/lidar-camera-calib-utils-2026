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
        msg.height = 1080
        msg.width = 1440
        msg.distortion_model = 'plumb_bob'
        msg.d = [-0.106887,0.098889,0.000438,-0.000049,0.000000]
        msg.k = [2359.228979,0.000000,706.732103, 0.000000,2355.309404,554.259586, 0.0, 0.0, 1.0]
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