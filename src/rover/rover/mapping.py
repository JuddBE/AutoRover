import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from math import sqrt, pi, cos, sin, atan2
from sympy import Plane, Point3D, Matrix, acos
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class PointCloudMapper(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')

        # Rover specifications used for traversability calculations.
        self.clearance_height = 1.0  # Height of rover in meters.
        self.max_pitch_angle = 30  # Max allowable pitch angle in degrees to be traversable (assuming everything else is perfect).

        # Odometry for rover position and orientation.
        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        # Map variables
        self.tile_size = 0.4  # Length of square tiles in meters.
        self.global_width = 150  # Number of tiles in global map's width.
        self.global_height = 150  # Number of tiles in global map's height.
        self.default_cost = 30
        self.cost_thresh = 0.6  # Any costs higher than 60 / 100 are deemed untraversable.
        self.recenter_dist = 10  # Number of tiles rover can be from edge before recentering global map.
        self.init_x_min_lim = self.tile_size * self.global_width / -2
        self.init_x_max_lim = self.init_x_min_lim * -1
        self.init_y_min_lim = self.tile_size * self.global_height / -2
        self.init_y_max_lim = self.init_y_min_lim * -1
        self.x_min_lim = self.init_x_min_lim
        self.x_max_lim = self.init_x_max_lim
        self.y_min_lim = self.init_y_min_lim
        self.y_max_lim = self.init_y_max_lim
        self.cost_map = np.full((self.global_width, self.global_height), self.default_cost, dtype=int)
        self.global_origin = (0.0, 0.0)
        self.metadata = MapMetaData()
        self.hazard_kernel = [[50, 70, 50],  # Sets the minimum cost for tiles surrounding hazards (100 cost tiles).
                              [70, 100, 70],
                              [50, 70, 50]]
        
        # Subscribe to the lidar point cloud topic.
        self.lidar_subscription = self.create_subscription(PointCloud2, '/cloud', self.pointcloud_callback, 1)

        # Subscribe to the odom topic.
        self.odom_subscription = self.create_subscription(Odometry, '/position', self.odom_callback, 1)

        # Subscribe to the signal mapping and path planning topic.
        self.signal_subscription = self.create_subscription(Bool, '/signal_mapping', self.publish_map, 1)

        # Publish map to trav_map topic.
        self.publisher_ = self.create_publisher(OccupancyGrid, 'trav_map', 1)

        # Publish new origin coords of map.
        self.origin_publisher = self.create_publisher(Point, 'origin', 1)

    def apply_kernel(self, i, j):
        kernel_len = len(self.hazard_kernel)
        for dy in range(kernel_len):
            for dx in range(kernel_len):
                nx = j + dx - kernel_len//2
                ny = i + dy - kernel_len//2
                if ny >= 0 and ny < self.global_height and nx >= 0 and nx < self.global_height:
                    self.cost_map[ny][nx] = max(self.cost_map[ny][nx], self.hazard_kernel[dy][dx])
                    
    def publish_map(self, msg):
        # Flatten the 2D grid into a 1D array.
        flat_map = [cell for row in self.cost_map.tolist() for cell in row]

        # Publish new origin point.
        origin_msg = Point()
        origin_msg.x = self.global_origin[0]
        origin_msg.y = self.global_origin[1]
        origin_msg.z = 0.0
        self.origin_publisher.publish(origin_msg)

        # Initialize metadata for the map.
        self.metadata.resolution = self.tile_size  # Size of tiles.
        self.metadata.width = self.global_width
        self.metadata.height = self.global_height
        self.metadata.origin.position.x = self.global_origin[0]
        self.metadata.origin.position.y = self.global_origin[1]
        self.metadata.origin.position.z = 0.0
        self.metadata.origin.orientation.w = 1.0

        # Create the OccupancyGrid message.
        grid_msg = OccupancyGrid()
        grid_msg.header = Header()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info = self.metadata
        grid_msg.data = flat_map

        # Publish the map.
        self.publisher_.publish(grid_msg)


    def pointcloud_callback(self, msg):
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        # Rotation matrix for roll.
        R_roll = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        # Rotation matrix for pitch.
        R_pitch = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        # Rotation matrix for yaw.
        R_yaw = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # Intrinsic rotation matrix: R_yaw * R_pitch * R_roll.
        R = np.matmul(np.matmul(R_yaw, R_pitch), R_roll)

        # self.get_logger().info(str(self.x) + " " + str(self.y) + " " + str(self.z) + " - " + str(self.roll) + " " + str(self.pitch) + " " + str(self.yaw))
        
        # Convert the PointCloud2 message to a list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        num_points = len(points)

        # Track mins and maxes for axes (used to normalize)
        x_min = float('inf')
        x_max = -float('inf')
        y_min = float('inf')
        y_max = -float('inf')
        z_min = float('inf')
        z_max = -float('inf')

        # Initialize tiles for 2D traversability map.
        tiles = [np.empty((0, 3)) for _ in range(self.global_width * self.global_height)]

        # Remember global rover position of scan.
        global_pos = np.array([self.x, self.y, self.z])
        rover_x_idx = int((global_pos[0] - self.x_min_lim) / self.tile_size)
        rover_y_idx = int((global_pos[1] - self.y_min_lim) / self.tile_size)


        # If near edge of global map, recenter around rover.
        if rover_x_idx < self.recenter_dist or rover_x_idx >= self.global_width - self.recenter_dist or rover_y_idx < self.recenter_dist or rover_y_idx >= self.global_height - self.recenter_dist:
            # Shift origin of cost map.
            new_origin_x = ((rover_x_idx - (self.global_width / 2)) * self.tile_size) + self.global_origin[0]
            new_origin_y = ((rover_y_idx - (self.global_height / 2)) * self.tile_size) + self.global_origin[1]
            self.global_origin = (new_origin_x, new_origin_y)

            # Track new x and y limits.
            self.x_max_lim = self.init_x_max_lim + new_origin_x
            self.x_min_lim = self.init_x_min_lim + new_origin_x
            self.y_max_lim = self.init_y_max_lim + new_origin_y
            self.y_min_lim = self.init_y_min_lim + new_origin_y

            # Shift actual cost map.
            x_shift = int(rover_x_idx - (self.global_width / 2))
            y_shift = int(rover_y_idx - (self.global_height / 2))
            new_cost_map = np.full_like(self.cost_map, self.default_cost)

            if x_shift >= 0:  # Shift left
                new_cost_map[:, :-x_shift] = self.cost_map[:, x_shift:]
                new_cost_map[:, -x_shift:] = self.default_cost  # Clear out right columns
            elif x_shift < 0:  # Shift right
                new_cost_map[:, -x_shift:] = self.cost_map[:, :x_shift]
                new_cost_map[:, :-x_shift] = self.default_cost  # Clear out left columns

            if y_shift > 0:  # Shift up
                new_cost_map[:-y_shift, :] = new_cost_map[y_shift:, :]
                new_cost_map[-y_shift:, :] = self.default_cost  # Clear out bottom rows
            elif y_shift < 0:  # Shift down
                new_cost_map[-y_shift:, :] = new_cost_map[:y_shift, :]
                new_cost_map[:-y_shift, :] = self.default_cost  # Clear out top rows

            self.cost_map = new_cost_map
        
        # Remove nonexistant points and find mins and maxes for x, y, and z (filtering point clouds).
        for i in range(num_points - 1, -1, -1):
            if abs(points[i][0]) == float('inf') or abs(points[i][1]) == float('inf') or abs(points[i][2]) == float('inf'):
                del points[i]
                continue

            # Find mins and maxes for x, y, z.
            if points[i][0] < x_min:
                x_min = points[i][0]
            if points[i][0] > x_max:
                x_max = points[i][0]

            if points[i][1] < y_min:
                y_min = points[i][1]
            if points[i][1] > y_max:
                y_max = points[i][1]

            if points[i][2] < z_min:
                z_min = points[i][2]
            if points[i][2] > z_max:
                z_max = points[i][2]

            # Convert local to global coords
            # Global rover position.
            local_point = np.array([points[i][0], points[i][1], points[i][2]])  # Make point numpy array for matrix mul

            # Apply rotation and global position/offset.
            R_point = np.matmul(R, local_point)
            global_point = R_point + global_pos

            # Update point
            points[i][0] = global_point[0]
            points[i][1] = global_point[1]
            points[i][2] = global_point[2]

            # Assign points to their tiles on the map.
            x_idx = int((points[i][0] - self.x_min_lim) / self.tile_size)
            y_idx = int((points[i][1] - self.y_min_lim) / self.tile_size)

            if x_idx < 0 or x_idx >= self.global_width or y_idx < 0 or y_idx >= self.global_height:
                # self.get_logger().info(f"Point out of bounds: x={points[i][0]}, y={points[i][1]}")
                # self.get_logger().info(f"Point out of bounds: x_idx={x_idx}, y_idx={y_idx}\n")
                del points[i]
                continue

            tile_idx = x_idx % self.global_width + y_idx * self.global_width
            tiles[tile_idx] = np.vstack([tiles[tile_idx], np.array([points[i][0], points[i][1], points[i][2]])])


        # Calculate 2D goodness map
        vec = np.array([0, 0, 1])
        for i in range(self.global_width * self.global_height):
            if len(tiles[i]) > 12:
                # Find best fit plane.
                # Compute the centroid.
                centroid = np.mean(tiles[i], axis=0)

                # Center the points.
                centered_points = tiles[i] - centroid
                
                # Perform SVD.
                _, _, vh = np.linalg.svd(centered_points)
                
                # The normal vector to the plane is the last row of vh (or last column of V^T).
                normal_vector = vh[-1]
                
                # Calculate distances of all points in a tile to the best fit plane.
                # Extract normal vector and point on the plane
                normal = np.array(normal_vector, dtype=float)

                # Compute plane equation: ax + by + cz + d = 0
                d = -np.dot(normal, centroid)

                # Calculate distances for all points in a vectorized manner
                distances = np.abs(np.dot(tiles[i], normal) + d) / np.linalg.norm(normal)

                max_elev_diff = tiles[i][:,2].max() - tiles[i][:,2].min()

                # Intitialize vectors for elevation angle.
                dot_product = vec.dot(normal)

                # Magnitudes of the vectors.
                magnitude1 = sqrt(vec.dot(vec))
                magnitude2 = sqrt(normal.dot(normal))

                # Find elevation angle.
                angle = acos(dot_product / (magnitude1 * magnitude2))

                elev_angle = angle * 180 / pi
                if elev_angle > 90:
                    elev_angle = 180 - elev_angle

                # self.get_logger().info(str(elev_angle))
                
                # Calculate RMSD between points to their location on the best fit plane.
                squared_error_sum = 0
                n = len(distances)
                for distance in distances:
                    squared_error_sum += distance * distance
                rmsd = sqrt(squared_error_sum / n)

                # Calculate goodness value using surface normal, min/max error, and RMSD.
                roughness_frac = 0.3 * self.clearance_height
                pitch_goodness = (1 - min(1, elev_angle / self.max_pitch_angle))
                if rmsd < roughness_frac:
                    rmsd_goodness = 1
                else:
                    rmsd_goodness = (1 - min(1, roughness_frac * rmsd / self.clearance_height))
                if max_elev_diff < 0.3 * self.clearance_height:
                    step_goodness = 0.5
                else:
                    step_goodness = 0.5 * (1 - min(1, max_elev_diff/self.clearance_height))

                cost = round(100 - (pitch_goodness + rmsd_goodness + step_goodness) * 40)

                if cost > 45:
                    cost = 100
                elif cost <= 8:
                    cost = 1

                self.cost_map[i // self.global_width][i % self.global_width] = cost
        
        for i in range(self.global_width):
            for j in range(self.global_height):
                if self.cost_map[i][j] == 100:
                    self.apply_kernel(i, j)


        # self.get_logger().info('Map Generated')

        # Publish the traversability map.
        # self.publish_map()


        # Graph goodness map
        # fig, ax = plt.subplots()

        # for i in range(self.global_width): 
        #     for j in range(self.global_height):
        #         # Calculate position of the square
        #         x = j * self.tile_size - self.global_width * self.tile_size / 2
        #         y = i * self.tile_size - self.global_height * self.tile_size / 2

        #         # Calculate square color
        #         if self.cost_map[i][j] <= 100:
        #             color_fac = 1 - (self.cost_map[i][j]) / (100)  # Color factor = 1, green | color factor = 0, red
        #             if color_fac < 0.5:
        #                 color = (1, color_fac*2, 0, 1)
        #             else:
        #                 color = (1 - 2*(color_fac - 0.5), 1, 0, 1)
        #         else:
        #             color = (0,0,0,1)
                
        #         # Create square
        #         square = plt.Rectangle((x, y), self.tile_size, self.tile_size, facecolor=color, edgecolor='black')
                
        #         # Add square to plot
        #         ax.add_patch(square)
        # # Set limits of the plot
        # ax.set_xlim(self.x_min_lim, self.x_max_lim)
        # ax.set_ylim(self.y_min_lim, self.y_max_lim)
        # plt.show()
    
    def odom_callback(self, msg):
        # Handle positional odometry.
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z

        # Handle orientation odometry.
        quaternion = msg.pose.pose.orientation
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Convert rotation to roll, pitch, yaw.
        self.roll, self.pitch, self.yaw = rotation.as_euler('xyz', degrees=False)
            

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

