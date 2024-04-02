import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import select
import termios
import tty

class TeleopKeyboard(Node):
    """
    A ROS2 node for teleoperating a robot using keyboard inputs.
    This node publishes Int32 messages based on the pressed key
    to control the robot's movement.
    """
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Int32, 'drivebase_subscriber', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.key = None
        print("Press 'w' to move forward, 's' to move backward, 'a' to turn left, 'd' to turn right, any key to stop current action, and 'q' to quit")

    def timer_callback(self):
        """
        Timer callback that checks for keyboard input and publishes the corresponding command.
        """
        self.get_key()
        msg = Int32()
        msg.data = 5 # stop
        if self.key:
            if self.key == 'q':
                sys.exit()
            if self.key == 'w':
                msg.data = 1  # Forward
            elif self.key == 's':
                msg.data = 2  # Backward
            elif self.key == 'a':
                msg.data = 3  # Left
            elif self.key == 'd':
                msg.data = 4  # Right
            else:
                msg.data = 5  # Stop
        self.publisher_.publish(msg)
        self.key = None

    def get_key(self):
        """
        Sets the terminal to raw mode to capture one key press at a time
        and reads the key press without requiring Enter to be pressed.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    teleop_keyboard_node = TeleopKeyboard()
    try:
        rclpy.spin(teleop_keyboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_keyboard_node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()