import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import yaml

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')

        # Load the camera calibration file
        camera_info_url = self.declare_parameter('camera_info_url', '').value
        self.camera_info = self.load_camera_info(camera_info_url)

        if self.camera_info is None:
            self.get_logger().error('Failed to load camera calibration data from %s. Exiting...' % camera_info_url)
            rclpy.shutdown()
            return

        # Publisher for the camera info
        self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
        
        # Timer to periodically publish camera info
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_camera_info)

    def load_camera_info(self, file_path):
        try:
            with open(file_path, 'r') as file:
                calib_data = yaml.safe_load(file)
                if calib_data is None:
                    self.get_logger().error('Empty YAML file: %s' % file_path)
                    return None

                camera_info = CameraInfo()
                camera_info.width = calib_data.get('image_width', 0)
                camera_info.height = calib_data.get('image_height', 0)
                
                # Populate the camera matrix (k field)
                camera_matrix = calib_data.get('camera_matrix', {}).get('data', [])
                assert len(camera_matrix) == 9, "Camera matrix must contain exactly 9 elements"
                camera_info.k = camera_matrix

                # Populate other fields
                camera_info.distortion_model = calib_data.get('distortion_model', '')
                camera_info.d = calib_data.get('distortion_coefficients', {}).get('data', [])
                camera_info.r = calib_data.get('rectification_matrix', {}).get('data', [])
                camera_info.p = calib_data.get('projection_matrix', {}).get('data', [])

                return camera_info

        except FileNotFoundError:
            self.get_logger().error('Camera calibration file not found: %s' % file_path)
        except Exception as e:
            self.get_logger().error('Error loading camera calibration data: %s' % str(e))

        return None

    def publish_camera_info(self):
        # Add timestamp to camera info message
        camera_info_msg = self.camera_info
        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = 'camera'
        
        self.publisher_.publish(camera_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
