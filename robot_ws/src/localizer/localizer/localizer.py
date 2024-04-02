import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomMerger(Node):
    """
    A ROS2 node to merge odometry information from two different sources:
    'tag_odom' and 'model_odom'. We use model_odom to get odometry at each
    timestep, and when we have apriltag detection information, we use this
    to correct our current 2D pose to mitigate drift.
    """
    def __init__(self):
        """
        Initialize the node, create subscriptions to odometry topics and a publisher
        for the merged odometry information.
        """
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
        self.latest_model_odom_pose = None
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
        """
        Callback for tag odometry messages.

        Parameters:
        - msg: Tag position and identity information.
        """
        self.latest_tag_odom_pose = msg.pose.pose.position
        self.publish_merged_odom(True)

    def model_odom_callback(self, msg):
        """
        Callback for feedforward differential steering model odometry messages.

        Parameters:
        - msg: The Odometry message from model predictions.
        """

        # initialize last pose
        if self.last_pose == None:
            self.last_pose = msg.pose.pose.position

        if self.latest_model_odom_pose == None:
            self.latest_model_odom_pose = msg.pose.pose.position

        # Calculate the position deltas from the model odometry
        self.model_odom_dx = msg.pose.pose.position.x - self.latest_model_odom_pose.x
        self.model_odom_dy = msg.pose.pose.position.y - self.latest_model_odom_pose.y

        # Update the latest model odometry pose and orientation
        self.latest_model_odom_pose = msg.pose.pose.position
        self.latest_model_odom_orientation = msg.pose.pose.orientation
        self.publish_merged_odom()

    def publish_merged_odom(self, use_tag=False):
        """
        Publish the merged odometry message, using weighted averages of positions
        from the tag and model odometry.

        Parameters:
        - use_tag (bool): Flag to indicate whether to use the tag odometry for
                          merging in this update.
        """
        if self.latest_tag_odom_pose is not None and self.latest_model_odom_pose is not None and self.latest_model_odom_orientation is not None:
            merged_odom_msg = Odometry()
            merged_odom_msg.header.stamp = self.get_clock().now().to_msg()
            merged_odom_msg.header.frame_id = 'world'
            merged_odom_msg.pose.pose.orientation = self.latest_model_odom_orientation

            # apply vehicle model update
            self.last_pose.x += self.model_odom_dx
            self.last_pose.y += self.model_odom_dy
            self.last_pose.z = 0.0  # Assuming a 2D world
            
            # check if we have apriltag information to correct 2D pose
            if use_tag:
                # Perform weighted average for x and y positions
                self.last_pose.x = (self.tag_weight * self.latest_tag_odom_pose.x +
                                                        self.model_weight * self.last_pose.x)
                self.last_pose.y = (self.tag_weight * self.latest_tag_odom_pose.y +
                                                        self.model_weight * self.last_pose.y)
            
            # update last position and publish merged odometry
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
