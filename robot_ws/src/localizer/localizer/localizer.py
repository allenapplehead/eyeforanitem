import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMerger(Node):
    def __init__(self):
        super().__init__('odom_merger')
        self.subscription_tag_odom = self.create_subscription(
            Odometry,
            '/tag_odom',
            self.tag_odom_callback,
            10)
        self.subscription_model_odom = self.create_subscription(
            Odometry,
            '/model_odom',
            self.model_odom_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

        # Initialize to store latest positions and orientation from both sources
        self.latest_tag_odom_pose = None
        self.latest_model_odom_pose = None  # Added to store position from model_odom
        self.latest_model_odom_orientation = None

        # last robot odometry pose
        self.last_pose = None

        # vehicle model position updates
        self.model_odom_dx = 0.0
        self.model_odom_dy = 0.0

        # Weights for averaging positions
        self.tag_weight = 0.75
        self.model_weight = 0.25

    def tag_odom_callback(self, msg):
        self.latest_tag_odom_pose = msg.pose.pose.position
        self.publish_merged_odom(True)

    def model_odom_callback(self, msg):

        if self.last_pose == None:
            self.last_pose = msg.pose.pose.position

        if self.latest_model_odom_pose == None:
            self.latest_model_odom_pose = msg.pose.pose.position

        self.model_odom_dx = msg.pose.pose.position.x - self.latest_model_odom_pose.x
        self.model_odom_dy = msg.pose.pose.position.y - self.latest_model_odom_pose.y

        self.latest_model_odom_pose = msg.pose.pose.position  # Correctly update the model's position
        self.latest_model_odom_orientation = msg.pose.pose.orientation
        self.publish_merged_odom()

    def publish_merged_odom(self, use_tag=False):
        # Ensure both position sources have been received
        if self.latest_tag_odom_pose is not None and self.latest_model_odom_pose is not None and self.latest_model_odom_orientation is not None:
            merged_odom_msg = Odometry()
            merged_odom_msg.header.stamp = self.get_clock().now().to_msg()
            merged_odom_msg.header.frame_id = 'world'
            merged_odom_msg.pose.pose.orientation = self.latest_model_odom_orientation

            # apply vehicle model update
            self.last_pose.x += self.model_odom_dx
            self.last_pose.y += self.model_odom_dy
            self.last_pose.z = 0.0  # Assuming a 2D world
            
            if use_tag:
                # Perform weighted average for x and y positions
                self.last_pose.x = (self.tag_weight * self.latest_tag_odom_pose.x +
                                                        self.model_weight * self.last_pose.x)
                self.last_pose.y = (self.tag_weight * self.latest_tag_odom_pose.y +
                                                        self.model_weight * self.last_pose.y)
            
            merged_odom_msg.pose.pose.position = self.last_pose
            self.publisher.publish(merged_odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_merger = OdomMerger()
    rclpy.spin(odom_merger)
    odom_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
