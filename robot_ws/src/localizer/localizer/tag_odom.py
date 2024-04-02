import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import yaml
import numpy as np
import os
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Quaternion
from ament_index_python.packages import get_package_share_directory


def rpy_from_quaternion(x, y, z, w):
    """
    Converts a quaternion orientation to roll, pitch, and yaw angles.

    Parameters:
    - x (float): x component of the quaternion.
    - y (float): y component of the quaternion.
    - z (float): z component of the quaternion.
    - w (float): w component of the quaternion.

    Returns:
    - list: A list containing the roll, pitch, and yaw angles in radians.
    """
    roll = np.arctan2(2.0 * (z * y + w * x), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(2.0 * (y * w - z * x))
    yaw = np.arctan2(2.0 * (z * w + x * y), - 1.0 + 2.0 * (w * w + x * x))

    return [roll, pitch, yaw]  # in radians


def quaternion_from_rpy(roll, pitch, yaw):
    """
    Converts roll, pitch, and yaw angles to a quaternion.

    Parameters:
    - roll (float): The roll angle in radians.
    - pitch (float): The pitch angle in radians.
    - yaw (float): The yaw angle in radians.

    Returns:
    - list: A list containing the quaternion components [w, x, y, z].
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - \
        np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - \
        np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + \
        np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qw, qx, qy, qz]


def calculate_rotation_matrix(x, y, z):
    """
    Calculates the rotation matrix from roll, pitch, and yaw angles.

    Parameters:
    - roll (float): The roll angle in radians.
    - pitch (float): The pitch angle in radians.
    - yaw (float): The yaw angle in radians.

    Returns:
    - np.ndarray: A 3x3 rotation matrix.
    """
    yaw = np.array([[np.cos(z), -np.sin(z),  0],
                    [np.sin(z), np.cos(z), 0],
                    [0,         0,          1]])

    pitch = np.array([[np.cos(y), 0, np.sin(y)],
                      [0, 1, 0],
                      [-np.sin(y), 0, np.cos(y)]])

    roll = np.array([[1, 0, 0],
                     [0, np.cos(x), -np.sin(x)],
                     [0, np.sin(x), np.cos(x)]])

    rotation_matrix = np.matmul(np.matmul(yaw, pitch), roll)

    return rotation_matrix


def create_transformation_matrix(rotation_matrix, translation_matrix):
    """
    Creates a 4x4 transformation matrix from rotation and translation.

    Parameters:
    - rotation_matrix (np.ndarray): A 3x3 rotation matrix.
    - translation_vector (list): A list of x, y, z translations.

    Returns:
    - np.ndarray: A 4x4 transformation matrix.
    """
    x, y, z = translation_matrix[0], translation_matrix[1], translation_matrix[2]
    p = np.array([[x], [y], [z], [1]])

    T1 = np.vstack([rotation_matrix, [0, 0, 0]])
    T = np.append(T1, p, 1)
    return T


class TagOdometryNode(Node):
    """
    ROS2 node for computing the odometry of a robot based on detected AprilTag poses.
    """
    def __init__(self):
        super().__init__('tag_odometry_node')

        self.tag_id = []
        self.tag_position = []
        self.tag_rotation = []
        self.tag_to_world = []
        self.camera_to_body = []

        # load parameters
        self.declare_parameter(
            'tags_config', '/workspace/robot_ws/src/localizer/config/tags.yaml')
        self.load_camera_extrinsics()

        # publishers and subscribers
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_detections_callback,
            10)

        self.pose_pub = self.create_publisher(Odometry, '/tag_odom', 10)

    def load_camera_extrinsics(self):
        """
        Loads camera extrinsic parameters and AprilTag configurations from a YAML file.
        """
        config_path = self.get_parameter(
            'tags_config').get_parameter_value().string_value

        try:
            with open(config_path, 'r') as file:
                data = yaml.safe_load(file)

                # load tag configs
                num_tags = data['num_tags']

                self.tag_id = [0] * num_tags
                self.tag_position = [0, 0, 0] * num_tags
                self.tag_rotation = [0, 0, 0] * num_tags
                self.tag_to_world = [[0.0, 0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0, 0.0],
                                     [0.0, 0.0, 0.0, 0.0]] * num_tags

                # Save data for each tag
                for i in range(num_tags):

                    tag_id_text = "tag_id_{}".format(i)
                    tag_position_text = "tag_position_{}".format(i)
                    tag_rotation_text = "tag_rotation_{}".format(i)

                    self.tag_id[i] = data[tag_id_text]
                    self.tag_position[i] = data[tag_position_text]
                    self.tag_rotation[i] = data[tag_rotation_text]

                    tag_rotation_matrix = calculate_rotation_matrix(self.tag_rotation[i][0]*np.pi/180,
                                                                    self.tag_rotation[i][1] *
                                                                    np.pi/180,
                                                                    self.tag_rotation[i][2]*np.pi/180)

                    # Tag in World Frame
                    self.tag_to_world[i] = create_transformation_matrix(
                        tag_rotation_matrix, self.tag_position[i])

                # load camera extringsics
                t_B_CB = data['t_B_CB']

                rotation_matrix = data['R_B_C']

                self.camera_to_body = create_transformation_matrix(
                    rotation_matrix, t_B_CB)

                print("Loaded tags and camera extrinsics")

        except FileNotFoundError as e:
            self.get_logger().error(f'Configuration file not found: {e}')
            self.destroy_node()

    def tag_detections_callback(self, msg):
        """
        Callback function for AprilTag detection messages. Processes the detections and publishes odometry information.

        Parameters:
        - msg (AprilTagDetectionArray): The detected AprilTags with their poses.
        """
        tags_detected = len(msg.detections)
        if tags_detected == 0:
            return

        body_to_world = []

        for detection in msg.detections:

            # Extract information from detected tag
            detected_id = detection.id

            pose = detection.pose.pose.pose
            detected_pos = np.array(
                [pose.position.x, pose.position.y, pose.position.z])
            detected_ori = rpy_from_quaternion(
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

            T_BW = self.calculate_drivebase_position(
                detected_id, detected_pos, detected_ori)

            body_to_world.append(T_BW)

        # We only use first tag detected (for now)
        T_BW = body_to_world[0]

        print("Publishing pose from tag: ", msg.detections[0].id)

        # Publish drone's pose
        self.publish_matrix(T_BW)

    def publish_matrix(self, transformation_matrix):
        """
        Publishes the computed odometry from the transformation matrix.

        Parameters:
        - transformation_matrix (np.ndarray): A 4x4 transformation matrix representing the robot's pose.
        """

        T = transformation_matrix

        yaw = np.arctan2(T[1][0], T[0][0])
        pitch = np.arctan2(-T[2][0], np.sqrt(T[2][1]**2+T[2][2]**2))
        roll = np.arctan2(T[2][1], T[2][2])

        p = Odometry()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'world'
        p.child_frame_id = 'base_link'

        p.pose.pose.position.x = T[0][3]
        p.pose.pose.position.y = T[1][3]
        p.pose.pose.position.z = 0.0 #  2D pose only

        w, x, y, z = quaternion_from_rpy(0.0, 0.0, yaw)  # 2D pose only

        p.pose.pose.orientation.x = x
        p.pose.pose.orientation.y = y
        p.pose.pose.orientation.z = z
        p.pose.pose.orientation.w = w

        self.pose_pub.publish(p)

    def calculate_drivebase_position(self, id, position, orientation):
        """
        Calculates the robot's position based on the detected tag and the camera-to-body transformation.

        Parameters:
        - tag_id (int): The ID of the detected tag.
        - position (list): The position of the detected tag in the camera frame.
        - orientation (list): The orientation of the detected tag in the camera frame.

        Returns:
        - np.ndarray: A 4x4 transformation matrix representing the robot's pose in the world frame.
        """
        # Find apriltag_to_camera, then apriltag_to_body using detection info and camera_to_body parameter
        rotation_matrix = calculate_rotation_matrix(
            orientation[0], orientation[1], orientation[2])
        apriltag_to_camera = create_transformation_matrix(
            rotation_matrix, position)

        apriltag_to_body = np.matmul(self.camera_to_body, apriltag_to_camera)

        # To calculate body_to_apriltag we have to inverse apriltag_to_body

        body_to_apriltag = np.array([[1.0, 0.0, 0.0, 0.0],
                                     [0.0, 1.0, 0.0, 0.0],
                                     [0.0, 0.0, 1.0, 0.0],
                                     [0.0, 0.0, 0.0, 1.0]])

        # Inverse of rotation is transpose
        body_to_apriltag[0:3, 0:3] = np.transpose(
            np.array(apriltag_to_body[0:3, 0:3]))

        # Inverse of position is negative of position multiply by inverse of rotation
        apriltag_to_body[0:3, 3] = -1*apriltag_to_body[0:3, 3]
        body_to_apriltag[0:3, 3] = np.matmul(
            body_to_apriltag[0:3, 0:3], apriltag_to_body[0:3, 3])

        # Find position of drone body in the world
        body_to_world = np.matmul(self.tag_to_world[id], body_to_apriltag)

        return body_to_world


def main(args=None):
    rclpy.init(args=args)
    node = TagOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
