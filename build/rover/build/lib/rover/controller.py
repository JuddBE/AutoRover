import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')

        # Controller variables
        self.max_vel = 2.0  # m/s
        self.vel = self.max_vel
        self.angular_vel = 0.0

        # Subscribe to the steer angle topic.
        self.steer_subscription = self.create_subscription(Point, '/steer_angle', self.steer_callback, 1)

        # Create a publisher on the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # Timer to periodically send messages
        # self.timer = self.create_timer(0.5, self.control)

    def control(self):
        # Create a twist message
        msg = Twist()
        msg.linear.x = self.vel  # Forward linear velocity
        msg.linear.y = 0.0  # Sideways velocity
        msg.linear.z = 0.0  # Vertical velocity
        msg.angular.x = 0.0  # Angular X rotation
        msg.angular.y = 0.0  # Angular Y rotation
        msg.angular.z = self.angular_vel  # Angular Z rotation (turning)
        
        # Publish message
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: Linear x={msg.linear.x}, Angular z={msg.angular.z}')

    def steer_callback(self, msg):
        steering_angle = msg.x
        move = (msg.y == 1.0)

        # self.get_logger().info(str(steering_angle))

        if move:
            if abs(steering_angle) > 0.03:
                self.vel = 0.0
                self.angular_vel = steering_angle * self.max_vel * 5
            else:
                self.vel = self.max_vel
                self.angular_vel = 0.0
        else:
            self.vel = 0.0
            self.angular_vel = 0.0

        self.control()

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

