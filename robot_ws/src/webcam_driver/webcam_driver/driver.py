import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os

class CameraDriver(Node):
    def __init__(self):
        super().__init__('webcam_driver_node')
        
        # Declare parameters with default values
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("topic_name", "camera/image_raw")
        self.declare_parameter("frame_rate", 30.0)
        
        # Retrieve parameters
        self.camera_id_ = self.get_parameter("camera_id").get_parameter_value().integer_value
        self.topic_name_ = self.get_parameter("topic_name").get_parameter_value().string_value
        self.frame_rate_ = self.get_parameter("frame_rate").get_parameter_value().double_value
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        
        # Initialize video capture
        self.cap = cv2.VideoCapture(self.camera_id_)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera with ID {self.camera_id_}')
            os._exit(1)
        
        # Set the capture frame rate
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate_)
        
        # Publisher for raw camera images
        self.publisher_ = self.create_publisher(Image, self.topic_name_, 10)
        
        # Setup timer to capture and publish frames
        timer_period = 1.0 / self.frame_rate_  # Convert FPS to period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera driver started, publishing to "{self.topic_name_}" at {self.frame_rate_} FPS.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Error capturing frame')
            return
        
        # Convert the OpenCV image to a ROS Image message and publish
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(ros_image)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
