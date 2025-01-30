import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
import numpy as np
from math import sqrt

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Current rover position.
        self.rover_x_idx = 0
        self.rover_y_idx = 0
        self.rover_pos = np.array([0.0,0.0])
        self.yaw = 0
        
        # Waypoint coords (converted from tile indices to coordinates in the center of a tile).
        # self.waypoints = np.array([[1.0,1.0],[1.0,0.0],[2.0,0.0],[3.0,1.0],[2.0,2.0],[1.0,1.0],[0.0,0.0],[-1.0,0.0]])
        self.waypoints = np.array([])
        self.x_idx = 0
        self.y_idx = 0

        # Map variables
        self.tile_size = 0.4  # Length of square tiles in meters.
        self.global_size = 150
        self.init_x_min_lim = self.tile_size * self.global_size / -2
        self.init_x_max_lim = self.init_x_min_lim * -1
        self.init_y_min_lim = self.tile_size * self.global_size / -2
        self.init_y_max_lim = self.init_y_min_lim * -1
        self.x_min_lim = self.init_x_min_lim
        self.x_max_lim = self.init_x_max_lim
        self.y_min_lim = self.init_y_min_lim
        self.y_max_lim = self.init_y_max_lim
        self.global_origin = np.array([0.0, 0.0])

        # Subscribe to the waypoint topic.
        self.waypoint_subscription = self.create_subscription(Float32MultiArray, '/waypoints', self.waypoint_callback, 1)

        # Subscribe to the odom topic.
        self.odom_subscription = self.create_subscription(Odometry, '/position', self.odom_callback, 1)

        # Subscribe to the origin topic.
        self.origin_subscription = self.create_subscription(Point, '/origin', self.origin_callback, 1)

        # Publish steering angle to topic for controller.
        self.steer_publisher = self.create_publisher(Point, 'steer_angle', 1)

        # Publish start signal to remap and replan path.
        self.signal_publisher = self.create_publisher(Bool, 'signal_mapping', 1)
        self.received_response = True

        # Timer to perform pure pursuit continously.
        self.timer = self.create_timer(0.1, self.pure_pursuit)

    def pure_pursuit_control(self, target):
        # Calculate desired steering angle for pure pursuit
        dx = target[0] - self.rover_pos[0]
        dy = target[1] - self.rover_pos[1]
        target_angle = np.arctan2(dy,dx)

        # self.get_logger().info(str(target_angle))

        # Calculate steering angle
        steering_angle = target_angle - self.yaw
        steering_angle = np.arctan2(np.sin(steering_angle), np.cos(steering_angle))  # Normalize to [-pi, pi]

        # self.get_logger().info(str(self.yaw))

        return steering_angle

    def pure_pursuit(self):
        # Initialize message to controller.
        msg = Point()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        # If in tile of desired waypoint, update map and pathing.
        if self.x_idx == self.rover_x_idx and self.y_idx == self.rover_y_idx and self.received_response:
            sig_msg = Bool()
            sig_msg.data = True
            self.signal_publisher.publish(sig_msg)
            self.received_response = False
            self.get_logger().info('Signal to pathing: (' + str(self.rover_x_idx) + "," + str(self.rover_y_idx) + ")")

        # Check if there are any more waypoints.
        if len(self.waypoints) <= 0:
            self.get_logger().info("No path to follow.")
            self.steer_publisher.publish(msg)
            self.received_response = True
            return

        # Check if next waypoint has been reached.
        if sqrt((self.waypoints[0][0] - self.rover_pos[0])**2 + (self.waypoints[0][1] - self.rover_pos[1])**2) <= self.tile_size / 2.2:
            self.get_logger().info(f"Waypoint reached: ({str(self.x_idx)},{str(self.y_idx)})")
            self.get_logger().info(f"Waypoint coord: ({str(self.waypoints[0][0])},{str(self.waypoints[0][1])})")
            self.waypoints = np.delete(self.waypoints, 0, axis=0)
            # self.get_logger().info(str(self.received_response))
            # self.get_logger().info(f"Waypoint reached: ({str(self.x_idx)},{str(self.y_idx)})")
            self.get_logger().info(f"Rover tiles: ({str(self.rover_x_idx)},{str(self.rover_y_idx)})")
            self.get_logger().info(f"Rover pos: ({str(self.rover_pos[0])},{str(self.rover_pos[1])})")

            # Check if there are any more waypoints.""
            if len(self.waypoints) <= 0:
                self.get_logger().info("No path to follow.")
                self.steer_publisher.publish(msg)
                return
            
            self.x_idx = int((self.waypoints[0][0] - self.x_min_lim) / self.tile_size)
            self.y_idx = int((self.waypoints[0][1] - self.y_min_lim) / self.tile_size)
        
        # Check if there are any more waypoints.""
        if len(self.waypoints) <= 0:
            self.get_logger().info("No path to follow.")
            self.steer_publisher.publish(msg)
            return

        steering_angle = self.pure_pursuit_control(self.waypoints[0])
        msg.x = steering_angle
        msg.y = 1.0
        self.steer_publisher.publish(msg)

    def odom_callback(self, msg):
        # Handle positional odometry.
        position = msg.pose.pose.position
        self.rover_pos[0] = position.x
        self.rover_pos[1] = position.y
        self.rover_x_idx = int((self.rover_pos[0] - self.x_min_lim) / self.tile_size)
        self.rover_y_idx = int((self.rover_pos[1] - self.y_min_lim) / self.tile_size)

        # rover_x_temp = int((self.rover_pos[0] - self.x_min_lim) / self.tile_size)
        # rover_y_temp = int((self.rover_pos[1] - self.y_min_lim) / self.tile_size)

        # If rover changes tile, update the coordinates and calculate a new path.
        # if self.rover_x_idx != rover_x_temp or self.rover_y_idx != rover_y_temp:
        #     self.rover_x_idx = rover_x_temp
        #     self.rover_y_idx = rover_y_temp
        #     sig_msg = Bool()
        #     sig_msg.data = True
        #     self.signal_publisher.publish(sig_msg)
        #     self.received_response = False
        #     self.get_logger().info('Signal to pathing: (' + str(self.rover_x_idx) + "," + str(self.rover_y_idx) + ")")

        # Handle orientation odometry.
        quaternion = msg.pose.pose.orientation
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Convert rotation to roll, pitch, yaw.
        _, _, self.yaw = rotation.as_euler('xyz', degrees=False)

    def origin_callback(self, msg):
        self.global_origin[0] = msg.x
        self.global_origin[1] = msg.y

        # Track new x and y limits.
        self.x_max_lim = self.init_x_max_lim + self.global_origin[0]
        self.x_min_lim = self.init_x_min_lim + self.global_origin[0]
        self.y_max_lim = self.init_y_max_lim + self.global_origin[1]
        self.y_min_lim = self.init_y_min_lim + self.global_origin[1]
    
    def waypoint_callback(self, msg):
        # Read waypoint messages, then convert from tiles to coordinates centered on tiles.
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        new_waypoints = np.array(msg.data, dtype=int).reshape((rows, cols))
        self.x_idx = new_waypoints[0][0]
        self.y_idx = new_waypoints[0][1]
        self.get_logger().info('Received from pathing: (' + str(self.x_idx) + "," + str(self.y_idx) + ")")

        # Convert from tiles to coordinates centered on tiles.
        self.waypoints = (new_waypoints - (self.global_size / 2)) * self.tile_size + self.tile_size / 2 + self.global_origin

        self.received_response = True


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

