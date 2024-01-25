import cv2
import os
import datetime
import rclpy
import rclpy.node

class ImageCollector(rclpy.node.Node):
    def __init__(self):
        super().__init__('image_collector_node')

        # Declare parameters
        self.declare_parameter("capture_period", 1.0)
        self.declare_parameter("dataset_dir", "/data/datasets/image_collector/train")
        self.declare_parameter("blurry_dir", "/data/datasets/image_collector/blurry")
        self.declare_parameter("save_imgs", True)
        self.declare_parameter("blur_thres", 100.0)

        # Get parameters
        self.capture_period_ = self.get_parameter("capture_period").get_parameter_value().double_value
        self.dataset_dir_ = self.get_parameter("dataset_dir").get_parameter_value().string_value
        self.blurry_dir_ = self.get_parameter("blurry_dir").get_parameter_value().string_value
        self.save_imgs_ = self.get_parameter("save_imgs").get_parameter_value().bool_value
        self.blur_thres_ = self.get_parameter("blur_thres").get_parameter_value().double_value

        print("+--------------------------+")
        print("Setting capture period to:", self.capture_period_)
        print("Setting dataset directory to:", self.dataset_dir_)
        print("Setting blurry image directory to:", self.blurry_dir_)
        print("Save images:", self.save_imgs_)
        print("Acceptable blur threshold:", self.blur_thres_)
        print("+--------------------------+")

        # Create dataset directory if it doesn't exist
        if not os.path.exists(self.dataset_dir_):
            try:
                os.makedirs(self.dataset_dir_, exist_ok=True)
                os.makedirs(self.blurry_dir_, exist_ok=True)
                self.get_logger().info(f"Created dataset directory: {self.dataset_dir_}")
            except OSError as e:
                self.get_logger().error(f"Failed to create dataset directory: {self.dataset_dir_}. Error: {e}")

        # Initialize video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture device')
            return

        # Create a timer
        self.timer = self.create_timer(self.capture_period_, self.timer_callback)

    def blurry_check(self, image):
        laplacian_var = cv2.Laplacian(image, cv2.CV_64F).var()
        return laplacian_var < self.blur_thres_

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

        # Check if images should be saved
        if self.save_imgs_:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = f"{timestamp}.png"
            if self.blurry_check(frame):
                filepath = os.path.join(self.blurry_dir_, filename)
            else:
                filepath = os.path.join(self.dataset_dir_, filename)
            cv2.imwrite(filepath, frame)
            self.get_logger().info(f"Saved image {filename}")

        # Display the current frame
        cv2.imshow('Current Frame', frame)
        cv2.waitKey(1)  # Add a brief wait to allow the image to be displayed

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = ImageCollector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
