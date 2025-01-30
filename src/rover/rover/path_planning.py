import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import PointStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
from rover.DStarLite import DStarLite

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # Map settings.
        self.global_width = 150
        self.global_height = 150
        self.cost_map = [[40] * self.global_width for _ in range(self.global_height)]
        self.tile_size = 0.4  # Length of square tiles in meters.
        self.init_x_min_lim = self.tile_size * self.global_width / -2
        self.init_x_max_lim = self.init_x_min_lim * -1
        self.init_y_min_lim = self.tile_size * self.global_height / -2
        self.init_y_max_lim = self.init_y_min_lim * -1
        self.x_min_lim = self.init_x_min_lim
        self.x_max_lim = self.init_x_max_lim
        self.y_min_lim = self.init_y_min_lim
        self.y_max_lim = self.init_y_max_lim
        self.global_origin = (0.0, 0.0)

        # Path settingss
        self.s_start = (0, 0)  # Rover tile coords.
        self.s_goal = (98, 75)  # Should eventually be found by selecting a tile in space.
        self.init_s_goal = self.s_goal
        self.temp_goal = self.s_goal  # Used to track temporary goals if final goal is outside of map.
        self.rover_pos = np.array([0.0,0.0])  # Rover coords.
        self.waypoints = []  # Path points.
        
        # Subscribe to the 2D traversability map topic.
        self.map_subscription = self.create_subscription(OccupancyGrid, '/trav_map', self.map_callback, 1)

        # Subscribe to the odom topic.
        self.odom_subscription = self.create_subscription(Odometry, '/position', self.odom_callback, 1)

        # Subscribe to the clicked point topic.
        self.goal_subscription = self.create_subscription(PointStamped, '/clicked_point', self.goal_callback, 1)

        # Publish waypoints to waypoints topic for path tracking.
        self.publisher_ = self.create_publisher(Float32MultiArray, 'waypoints', 1)

    def update_start(self):
        # Assign rover coords to their tiles on the map.
        x_idx = int((self.rover_pos[0] - self.x_min_lim) / self.tile_size)
        y_idx = int((self.rover_pos[1] - self.y_min_lim) / self.tile_size)
        self.s_start = (x_idx, y_idx)
    
    # def find_goal_in_boundary(self, x2, y2):
    #     x_min = self.global_width // -2
    #     x_max = self.global_width // 2
    #     y_min = self.global_height // -2
    #     y_max = self.global_height // 2
    #     x1 = self.s_start[0]
    #     y1 = self.s_start[1]
    #     # x2 = self.s_goal[0]
    #     # y2 = self.s_goal[1]

    #     intersections = []

    #     # Left boundary (x = x_min)
    #     if x2 != x1:
    #         t = (x_min - x1) / (x2 - x1)
    #         y = y1 + t * (y2 - y1)
    #         if t > 0 and y_min <= y <= y_max:
    #             intersections.append((t, (x_min, y)))

    #     # Right boundary (x = x_max)
    #     if x2 != x1:
    #         t = (x_max - x1) / (x2 - x1)
    #         y = y1 + t * (y2 - y1)
    #         if t > 0 and y_min <= y <= y_max:
    #             intersections.append((t, (x_max, y)))

    #     # Bottom boundary (y = y_min)
    #     if y2 != y1:
    #         t = (y_min - y1) / (y2 - y1)
    #         x = x1 + t * (x2 - x1)
    #         if t > 0 and x_min <= x <= x_max:
    #             intersections.append((t, (x, y_min)))

    #     # Top boundary (y = y_max)
    #     if y2 != y1:
    #         t = (y_max - y1) / (y2 - y1)
    #         x = x1 + t * (x2 - x1)
    #         if t > 0 and x_min <= x <= x_max:
    #             intersections.append((t, (x, y_max)))

    #     # Find the closest intersection
    #     if intersections:
    #         _, closest_point = min(intersections, key=lambda item: item[0])
    #         self.get_logger().info("Temp goal: " + str(closest_point))
    #         return closest_point
    #     self.get_logger().info("Nope")
    #     return None  # No valid intersection

    def plan_path(self):
        self.update_start()

        if self.s_start == self.s_goal:
            self.get_logger().info("At goal destination.")
            return

        self.get_logger().info("Signal received: " + str(self.s_start))
        self.get_logger().info(str(self.s_goal))
        self.DStarL = DStarLite(self.s_start, self.s_goal, self.global_width, self.global_height, self.cost_map)
        self.DStarL.initDStar()
        self.DStarL.computeShortestPath()

        new_waypoints = []

        while self.DStarL.s_start != self.DStarL.s_goal:
            if self.DStarL.grid[self.DStarL.s_start[1]][self.DStarL.s_start[0]].rhs == float('inf'):
                self.get_logger().info("No known path.")
                break
            
            # Update start (next tile to move to)
            min_s = float('inf')
            arg_min = None
            for s_ in self.DStarL.grid[self.DStarL.s_start[1]][self.DStarL.s_start[0]].neighbors:
                s_ = tuple(map(int, s_[1:-1].split(', '))) # convert string back to tuple of coords

                temp = self.DStarL.grid[self.DStarL.s_start[1]][self.DStarL.s_start[0]].neighbors[str(s_)] + self.DStarL.grid[s_[1]][s_[0]].g
                if temp < min_s:
                    min_s = temp
                    arg_min = s_
            self.DStarL.s_start = arg_min

            new_waypoints.append([self.DStarL.s_start[0], self.DStarL.s_start[1]])
        
        self.get_logger().info("Sending waypoint: " + str(new_waypoints[0]))
        self.waypoints = new_waypoints

        self.publish_waypoints()
    

    # Updates the map and causes a new path to be planned.
    def map_callback(self, msg):
        # Calculate shift in map to apply to goal tile coords.
        x_shift = int((msg.info.origin.position.x - self.global_origin[0]) / self.tile_size)
        y_shift = int((msg.info.origin.position.y - self.global_origin[1]) / self.tile_size)

        # Extract map data.
        self.global_width = msg.info.width
        self.global_height = msg.info.height
        self.global_origin = (msg.info.origin.position.x, msg.info.origin.position.y)  #  Absolute global coords of origin point in map.

        # Track new x and y limits.
        self.x_max_lim = self.init_x_max_lim + self.global_origin[0]
        self.x_min_lim = self.init_x_min_lim + self.global_origin[0]
        self.y_max_lim = self.init_y_max_lim + self.global_origin[1]
        self.y_min_lim = self.init_y_min_lim + self.global_origin[1]
        
        if x_shift != 0 or y_shift != 0:
            self.s_goal = (self.s_goal[0] - x_shift, self.s_goal[1] - y_shift)
        
        flat_grid = msg.data

        # Reconstruct 2D map.
        self.new_cost_map = [list(flat_grid[i * self.global_width:(i + 1) * self.global_width]) for i in range(self.global_height)]

        self.cost_map = self.new_cost_map

        self.plan_path()
    
    def publish_waypoints(self):
        # Flatten the 2D grid into a 1D array.
        num_points = len(self.waypoints)
        flat_waypoints = [column for row in self.waypoints for column in row]

        # Create the OccupancyGrid message.
        msg = Float32MultiArray()
        dim1 = MultiArrayDimension(label='rows', size=num_points, stride=2)
        dim2 = MultiArrayDimension(label='columns', size=2, stride=1)
        msg.layout.dim = [dim1, dim2]
        msg.data = flat_waypoints

        # Publish the map.
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        # Handle positional odometry.
        position = msg.pose.pose.position
        self.rover_pos[0] = position.x
        self.rover_pos[1] = position.y
    
    def goal_callback(self, msg):
        x_idx = int((msg.point.x - self.x_min_lim) / self.tile_size)
        y_idx = int((msg.point.y - self.y_min_lim) / self.tile_size)

        self.s_goal = (x_idx, y_idx)
        # self.find_goal_in_boundary(x_idx, y_idx)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()