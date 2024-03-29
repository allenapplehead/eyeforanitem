import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
import os
import datetime
import time
import pickle
import math
import shutil


def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class ImageCollector(Node):
    def __init__(self):
        super().__init__('image_collector_node')

        # Declare parameters
        self.declare_parameter("capture_period", 1.0)
        self.declare_parameter(
            "dataset_dir", "/data/datasets/image_collector/train")
        self.declare_parameter(
            "blurry_dir", "/data/datasets/image_collector/blurry")
        self.declare_parameter(
            "stamps_dir", "/data/datasets/image_collector/stamps")
        self.declare_parameter("save_imgs", True)
        self.declare_parameter("blur_thres", 100.0)
        self.declare_parameter("wipe_prev", False)
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("odom_topic", "/odom")

        # Get parameters
        self.capture_period_ = self.get_parameter(
            "capture_period").get_parameter_value().double_value
        self.dataset_dir_ = self.get_parameter(
            "dataset_dir").get_parameter_value().string_value
        self.blurry_dir_ = self.get_parameter(
            "blurry_dir").get_parameter_value().string_value
        self.stamps_dir_ = self.get_parameter(
            "stamps_dir").get_parameter_value().string_value
        self.save_imgs_ = self.get_parameter(
            "save_imgs").get_parameter_value().bool_value
        self.blur_thres_ = self.get_parameter(
            "blur_thres").get_parameter_value().double_value
        self.wipe_prev_ = self.get_parameter(
            "wipe_prev").get_parameter_value().bool_value

        # location and timestamp dictionary, to be pickled
        # (image name ->  (robot x, robot y, robot yaw))
        # note that the image name is already the timestamp
        self.stamp_dict = {}

        # last save time
        self.last_save_time_ = time.time()

        # most recent odometry
        self.last_odom_ = (0.0, 0.0, 0.0)  # zero initial state assumption

        print("+--------------------------+")
        print("Setting capture period to:", self.capture_period_)
        print("Setting dataset directory to:", self.dataset_dir_)
        print("Setting blurry image directory to:", self.blurry_dir_)
        print("Save images:", self.save_imgs_)
        print("Acceptable blur threshold:", self.blur_thres_)
        print("Wipe previous images:", self.wipe_prev_)
        print("Subscribing to image topic:", self.get_parameter(
            "image_topic").get_parameter_value().string_value)
        print("Subscribing to odometry topic:", self.get_parameter(
            "odom_topic").get_parameter_value().string_value)
        print("+--------------------------+")

        # Wipe previous images if requested
        if self.wipe_prev_:
            if os.path.exists(self.dataset_dir_):
                shutil.rmtree(self.dataset_dir_)
            if os.path.exists(self.blurry_dir_):
                shutil.rmtree(self.blurry_dir_)
            if os.path.exists(self.stamps_dir_):
                shutil.rmtree(self.stamps_dir_)
            self.get_logger().info("Wiped previous data")

        # Create dataset directory if it doesn't exist
        if not os.path.exists(self.dataset_dir_):
            os.makedirs(self.dataset_dir_, exist_ok=True)
        if not os.path.exists(self.blurry_dir_):
            os.makedirs(self.blurry_dir_, exist_ok=True)
        if not os.path.exists(self.stamps_dir_):
            os.makedirs(self.stamps_dir_, exist_ok=True)


        self.cv_bridge = CvBridge()

        # Subscribe to the image topic
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter(
                "image_topic").get_parameter_value().string_value,
            self.image_callback,
            10)

        # Subscribe to robot odometry
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter(
                "odom_topic").get_parameter_value().string_value,
            self.odom_callback,
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
                self.stamp_dict[filename] = self.last_odom_
                # write pickle to disc in dataset dir
                with open(os.path.join(self.stamps_dir_, "stamps.pkl"), 'wb') as f:
                    pickle.dump(self.stamp_dict, f)

            cv2.imwrite(filepath, cv_image)
            self.last_save_time_ = time.time()
            self.get_logger().info(f"Saved image {filename}")

        # Display the current frame (optional)
        cv2.imshow('Current Frame', cv_image)
        cv2.waitKey(1)

    def odom_callback(self, msg):
        # convert quat to euler
        x, y, z, w = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        roll, pitch, yaw = quaternion_to_euler(x, y, z, w)

        # Save the most recent 2D odometry of the robot
        self.last_odom_ = (msg.pose.pose.position.x,
                           msg.pose.pose.position.y, yaw)

    def __del__(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    image_collector = ImageCollector()
    rclpy.spin(image_collector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
