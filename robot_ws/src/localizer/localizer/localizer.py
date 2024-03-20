import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Twist, Vector3
import numpy as np
import time

    
class IMUToOdometry(Node):
    def __init__(self):
        super().__init__('imu_to_odometry')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile=qos_profile_sensor_data)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)
        
        self.last_time = time.time()
        self.position = Vector3()
        self.velocity = Vector3()
        self.yaw = 0.0

    def euler_to_quaternion(self, roll, pitch, yaw):
        # assume zero roll and pitch (2D world)
        roll = 0.0
        pitch = 0.0

        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def imu_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        ## Just orientation is quite stable. Position estimates are very drifty, unusable
        # # Assuming msg.linear_acceleration is in m/s^2
        # acc_x = msg.linear_acceleration.x
        # acc_y = msg.linear_acceleration.y
        
        # # Update velocity
        # self.velocity.x += acc_x * dt
        # self.velocity.y += acc_y * dt
        
        # # Update position
        # self.position.x += self.velocity.x * dt
        # self.position.y += self.velocity.y * dt

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position = self.position
        odom_msg.pose.pose.orientation = msg.orientation
        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()