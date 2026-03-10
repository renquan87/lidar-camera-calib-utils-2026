import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 从 yaml 文件加载标定参数
        calib_file = self.declare_parameter(
            'calib_file',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'parameters', 'ost.yaml')
        ).get_parameter_value().string_value

        self.camera_info_msg = self._load_camera_info(calib_file)
        self.get_logger().info(f'Camera Info Publisher has been started. Loaded from: {calib_file}')

    def _load_camera_info(self, file_path):
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        msg = CameraInfo()
        msg.header.frame_id = 'camera_frame'
        msg.height = data['image_height']
        msg.width = data['image_width']
        msg.distortion_model = data['distortion_model']
        msg.d = [float(x) for x in data['distortion_coefficients']['data']]
        msg.k = [float(x) for x in data['camera_matrix']['data']]
        msg.r = [float(x) for x in data['rectification_matrix']['data']]
        msg.p = [float(x) for x in data['projection_matrix']['data']]
        return msg

    def timer_callback(self):
        self.camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.camera_info_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_info_publisher = CameraInfoPublisher()
    rclpy.spin(camera_info_publisher)
    camera_info_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
