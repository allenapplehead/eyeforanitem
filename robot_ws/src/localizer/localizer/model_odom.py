import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose, Twist, Vector3
from std_msgs.msg import Int32
import numpy as np
import time
import math

    
class IMUToOdometry(Node):
    def __init__(self):
        super().__init__('imu_to_odometry')
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile=qos_profile_sensor_data)
        self.command_sub = self.create_subscription(
            Int32,
            '/drivebase_subscriber',
            self.command_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, '/model_odom', 10)
        
        self.last_time = time.time()
        self.position = Vector3()
        self.velocity = Vector3()

        # differential drive model
        self.b = 0.129 # wheel base [m]
        self.r = 0.031 # wheel radius [m]
        self.nu_l = 0.0 # revolutions per second [1/s]
        self.nu_r = 0.0 # revolutions per second [1/s]
        self.a = 2 * np.pi * self.r
        
        # zero state initial conditions (but use the current imu yaw reading)
        self.x_k = 0.0
        self.y_k = 0.0 
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

    def quaternion_to_euler(self, x, y, z, w):
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


    def command_callback(self, msg):
        # determine if the robot is moving:
        # forward: 1
        # backward: 2

        if msg.data == 1:
            self.nu_l = 1.0
            self.nu_r = 1.0
        elif msg.data == 2:
            self.nu_l = -1.0
            self.nu_r = -1.0
        else:
            self.nu_l = 0.0
            self.nu_r = 0.0
        
        return

    def imu_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # calculate x and y position
        _, _, self.yaw = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

        # imu has z axis pointing downwards
        self.yaw *= -1
        msg.orientation = self.euler_to_quaternion(0, 0, self.yaw)  # update msg.orientation

        self.x_k = self.x_k + (self.a * (self.nu_l + self.nu_r) * np.cos(self.yaw) * dt) / 2.0
        self.y_k = self.y_k + (self.a * (self.nu_l + self.nu_r) * np.sin(self.yaw) * dt) / 2.0

        self.position.x = self.x_k
        self.position.y = self.y_k
        self.position.z = 0.0

        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'world'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.position.x
        odom_msg.pose.pose.position.y = self.position.y
        odom_msg.pose.pose.position.z = self.position.z
        odom_msg.pose.pose.orientation = msg.orientation
        self.publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUToOdometry()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()