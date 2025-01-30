import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        # Create a publisher on the 'cmd_vel' topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # Timer to periodically send messages
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Create a Twist message
        msg = Twist()
        msg.linear.x = 0.5  # Forward linear velocity
        msg.linear.y = 0.0  # No sideways motion
        msg.linear.z = 0.0  # No vertical motion
        msg.angular.x = 0.0  # No angular rotation around X
        msg.angular.y = 0.0  # No angular rotation around Y
        msg.angular.z = 0.3  # Angular rotation around Z (turning)
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear x={msg.linear.x}, Angular z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

