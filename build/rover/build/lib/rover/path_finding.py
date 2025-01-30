import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
import numpy as np


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.cost_map = None
        self.global_width = 120
        self.global_height = 120

        # Subscribe to the 2D traversability map topic.
        self.map_subscription = self.create_subscription(OccupancyGrid, '/trav_map', self.map_callback, 1)
    
    # Updates the map and causes a new path to be planned.
    def map_callback(self, msg):
        # Extract map data.
        self.global_width = msg.info.width
        self.global_height = msg.info.height
        flat_grid = msg.data

        # Reconstruct 2D map.
        self.cost_map = [flat_grid[i * self.global_width:(i + 1) * self.global_width] for i in range(self.global_height)]

        self.get_logger().info(f'Received Grid: {self.cost_map}')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()