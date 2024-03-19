import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import os
import datetime
import time

class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector_node')
        
        # Declare parameters
        self.declare_parameter("capture_period", 1.0)
        self.declare_parameter("dataset_dir", "/data/datasets/image_collector/train")
        self.declare_parameter("blurry_dir", "/data/datasets/image_collector/blurry")
        self.declare_parameter("save_imgs", True)
        self.declare_parameter("blur_thres", 100.0)
        self.declare_parameter("wipe_prev", False)

        # Get parameters
        self.capture_period_ = self.get_parameter("capture_period").get_parameter_value().double_value
        self.dataset_dir_ = self.get_parameter("dataset_dir").get_parameter_value().string_value
        self.blurry_dir_ = self.get_parameter("blurry_dir").get_parameter_value().string_value
        self.save_imgs_ = self.get_parameter("save_imgs").get_parameter_value().bool_value
        self.blur_thres_ = self.get_parameter("blur_thres").get_parameter_value().double_value
        self.wipe_prev_ = self.get_parameter("wipe_prev").get_parameter_value().bool_value

        self.last_save_time_ = time.time()

        print("+--------------------------+")
        print("Setting capture period to:", self.capture_period_)
        print("Setting dataset directory to:", self.dataset_dir_)
        print("Setting blurry image directory to:", self.blurry_dir_)
        print("Save images:", self.save_imgs_)
        print("Acceptable blur threshold:", self.blur_thres_)
        print("Wipe previous images:", self.wipe_prev_)
        print("+--------------------------+")

        # Create dataset directory if it doesn't exist
        if not os.path.exists(self.dataset_dir_):
            os.makedirs(self.dataset_dir_, exist_ok=True)
            os.makedirs(self.blurry_dir_, exist_ok=True)
            self.get_logger().info(f"Created dataset directory: {self.dataset_dir_}")

        # Wipe previous images if requested
        if self.wipe_prev_:
            for file in os.listdir(self.dataset_dir_):
                os.remove(os.path.join(self.dataset_dir_, file))
            for file in os.listdir(self.blurry_dir_):
                os.remove(os.path.join(self.blurry_dir_, file))
            self.get_logger().info("Wiped previous images")

        self.cv_bridge = CvBridge()

        # Subscribe to the /camera/image_raw topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)

    def blurry_check(self, image):
        laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
        return laplacian_var < self.blur_thres_

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image as before
        if self.save_imgs_ and (time.time() - self.last_save_time_) > self.capture_period_:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{timestamp}.png"
            if self.blurry_check(cv_image):
                filepath = os.path.join(self.blurry_dir_, filename)
            else:
                filepath = os.path.join(self.dataset_dir_, filename)
            cv2.imwrite(filepath, cv_image)
            self.last_save_time_ = time.time()
            self.get_logger().info(f"Saved image {filename}")

        # Display the current frame (optional)
        cv2.imshow('Current Frame', cv_image)
        cv2.waitKey(1)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    image_collector = ImageCollector()
    rclpy.spin(image_collector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
