import cv2
import rclpy
import rclpy.node

class ImageCollector(rclpy.node.Node):
    def __init__(self):
        super().__init__('image_collector_node')

        # Declare parameters
        self.declare_parameter("capture_period", 1.0)  # Default values
        self.declare_parameter("dataset_dir", "/data/datasets/image_collector/train")

        # Get parameters
        self.capture_period_ = self.get_parameter("capture_period").get_parameter_value().double_value
        self.dataset_dir_ = self.get_parameter("dataset_dir").get_parameter_value().string_value

        print("Setting capture period to:", self.capture_period_)
        print("Setting dataset directory to:", self.dataset_dir_)

        # Initialize video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video capture device')
            return

        # Create a timer
        self.timer = self.create_timer(self.capture_period_, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return

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
