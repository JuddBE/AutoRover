"""
Extended Kalman Filter which fuses wheel odometry and IMU data to estimate rover pose.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
import numpy as np

class Localize(Node):
    def __init__(self):
        super().__init__('localization')
        
         # State vector [x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz]
        self.state = np.zeros(12)

        # Covariance matrix
        # self.P = np.eye(12) * 0.1
        self.P = np.diag([
            0.5, 0.5, 0.5,  # Position (x, y, z)
            0.001, 0.001, 0.001,  # Orientation (roll, pitch, yaw)
            1.0, 1.0, 1.0,  # Linear velocity (vx, vy, vz)
            0.001, 0.001, 0.001  # Angular velocity (wx, wy, wz)
        ])

        # Process noise
        # self.Q = np.eye(12) * 0.02
        self.Q = np.diag([
            2.0, 2.0, 2.0,  # Position (x, y, z)
            0.03, 0.03, 0.03,  # Orientation (roll, pitch, yaw)
            0.9, 0.9, 0.9,  # Linear velocity (vx, vy, vz)
            0.015, 0.015, 0.015  # Angular velocity (wx, wy, wz)
        ])

        # Measurement noise
        self.R = np.diag([
            2.0, 2.0, 2.0,  # x, y, z (wheel odom)
            0.03, 0.03, 0.03,  # roll, pitch, yaw (IMU)
            0.9, 0.9, 0.9,  # vx, vy, vz (wheel odom)
            0.015, 0.015, 0.015  # wx, wy, wz (IMU)
        ])

        # Identity matrix
        self.I = np.eye(12)

        # Latest sensor inputs
        self.latest_odom = None
        self.latest_imu = None
        self.fused_odom = np.zeros(12)

        # Subscribe to the odom topic.
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)

        # Subscribe to the IMU topic.
        self.imu_subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 1)

        # Create a publisher on the cmd_vel topic
        self.publisher_ = self.create_publisher(Odometry, 'position', 1)

        # Timer to periodically send messages
        self.dt = 0.01
        self.timer = self.create_timer(self.dt, self.publish_fused_odom)

    def predict(self):
        # Update state estimation using motion model
        # Update position using velocities
        self.state[0] += self.state[6] * self.dt
        self.state[1] += self.state[7] * self.dt
        self.state[2] += self.state[8] * self.dt

        # Update orientation using angular velocities
        self.state[3] += self.state[9] * self.dt
        self.state[4] += self.state[10] * self.dt
        self.state[5] += self.state[11] * self.dt

        F = np.eye(12)
        F[0, 6] = self.dt  # x - v_x
        F[1, 7] = self.dt  # y - v_y
        F[2, 8] = self.dt  # z - v_z
        F[3, 9] = self.dt  # Roll - wx
        F[4, 10] = self.dt # Pitch - w_y
        F[5, 11] = self.dt # Yaw - wz

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self):
        H = np.zeros((12, 12))
        H[0:3, 0:3] = np.eye(3)  # x, y, z
        H[3:6, 3:6] = np.eye(3)  # roll, pitch, yaw
        H[6:9, 6:9] = np.eye(3)  # vx, vy, vz
        H[9:12, 9:12] = np.eye(3)  # wx, wy, wz

        # Compute the measurement residual (y)
        y = self.fused_odom - (H @ self.state)

        # Compute the innovation covariance (S)
        S = H @ self.P @ H.T + self.R

        # Compute the Kalman Gain (K)
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update the state estimate
        self.state = self.state + K @ y

        # Update the covariance
        I = np.eye(12)
        self.P = (I - K @ H) @ self.P

    def publish_fused_odom(self):
        if self.latest_odom is None or self.latest_imu is None:
            return

        # EKF predict using wheel odom and IMU data for orientation.
        self.predict()

        # EKF Update using IMU data.
        self.update()

        # Construct filtered odom message.
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = self.state[0]
        msg.pose.pose.position.y = self.state[1]
        msg.pose.pose.position.z = self.state[2]
        msg.pose.covariance = self.P[:6, :6].flatten()

        # Convert roll, pitch, yaw to quaternion
        rotation = R.from_euler('xyz', [self.state[3], self.state[4], self.state[5]], degrees=False)
        quaternion = rotation.as_quat()
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        msg.twist.twist.linear.x = self.state[6]
        msg.twist.twist.linear.y = self.state[7]
        msg.twist.twist.linear.z = self.state[8]
        msg.twist.twist.angular.x = self.state[9]
        msg.twist.twist.angular.y = self.state[10]
        msg.twist.twist.angular.z = self.state[11]
        msg.twist.covariance = self.P[6:, 6:].flatten()

        # Publish filtered odometry.
        self.publisher_.publish(msg)

    def odom_callback(self, msg):
        # Replace odom orientation with imu orientation and publish.
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        # Save latest wheel odometry and state estimate.
        self.latest_odom = np.array([x, y, z, vx, vy, vz])
        self.fused_odom[0] = x
        self.fused_odom[1] = y
        self.fused_odom[2] = z
        self.fused_odom[6] = vx
        self.fused_odom[7] = vy
        self.fused_odom[8] = vz

    def imu_callback(self, msg):
        # Handle orientation odometry.
        quaternion = msg.orientation
        rotation = R.from_quat([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        
        # Convert rotation to roll, pitch, yaw.
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)

        # Update angular velocities
        wx = msg.angular_velocity.x
        wy = msg.angular_velocity.y
        wz = msg.angular_velocity.z

        # Save latest imu data
        self.latest_imu = np.array([roll, pitch, yaw, wx, wy, wz])
        self.fused_odom[3] = roll
        self.fused_odom[4] = pitch
        self.fused_odom[5] = yaw
        self.fused_odom[9] = wx
        self.fused_odom[10] = wy
        self.fused_odom[11] = wz

        # Use covariance from the IMU message (not necessary for now)
        # imu_orientation_cov = np.array(msg.orientation_covariance).reshape(3, 3)
        # imu_angular_velocity_cov = np.array(msg.angular_velocity_covariance).reshape(3, 3)

        # # Update measurement noise matrix
        # self.R = np.block([
        #     [imu_orientation_cov, np.zeros((3, 3))],
        #     [np.zeros((3, 3)), imu_angular_velocity_cov]
        # ])

def main(args=None):
    rclpy.init(args=args)
    node = Localize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
