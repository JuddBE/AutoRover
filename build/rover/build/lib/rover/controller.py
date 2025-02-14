"""
Directly sends movement commands (cmd_vel) to rover. Uses steering angle found from pure pursuit path tracking node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')

        # Controller variables
        self.max_vel = 0.5  # m/s
        self.max_turn_vel = 0.3  # rad/s
        self.vel = self.max_vel
        self.angular_vel = 0.0

        # Subscribe to the steer angle topic.
        self.steer_subscription = self.create_subscription(Point, '/steer_angle', self.steer_callback, 1)

        # Create a publisher on the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

    def control(self):
        """
        Directly controls the rover by publishing to cmd_vel topic.
        """

        # Create a twist message
        msg = Twist()
        msg.linear.x = self.vel  # Forward linear vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_vel  # Angular Z rotation (turning)
        
        # Publish message
        self.publisher_.publish(msg)

    def steer_callback(self, msg):
        """
        Using steering angle from pure pursuit, calculates the angular velocity and linear velocity. No PID to be found.
        """

        steering_angle = msg.x
        move = (msg.y == 1.0)

        if move:
            self.angular_vel = 0.0
            if steering_angle > 0.1:
                self.angular_vel = min(steering_angle - 0.07, self.max_turn_vel)
            else:
                self.angular_vel = max(steering_angle + 0.07, self.max_turn_vel * -1)

            # Prevent small linear velocities when turning, it can cause odometry to ruin faster.
            scale_factor = 1 - abs(self.angular_vel) / self.max_turn_vel
            self.vel = self.max_vel * scale_factor
            if self.vel < 0.05:
                self.vel = 0.0
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

