# AutoRover
Most aspects of the project are hand-made, including self-made code for SLAM, sensor fusion (EKF), navigation, path planning, and path tracking/controls. All these aspects were done without the use of plugins, including SLAM and navigation, to prioritize practical understanding of these areas outside of just ROS2 implementation.

Here is a very simplified rundown of what was implemented:

## Sensors
- 3D Lidar
- IMU
- Wheel encoders

## Localization
Uses an extended kalman filter to fuse wheel odometry and IMU data to create improved pose estimates. No lidar odometry has been applied (yet).

## Mapping
Using odometry, 3D point clouds from lidar are transformed to global points from local points. The 2D traversability map applies heuristics reliant on slopes, height differences, and surface variations/bumps to calculate cost, where each tile in the map is then filled with that cost value until the next update.

## Path planning
A D* lite algorithm is applied to plan and replan paths efficiently. Finds the lowest-cost path to get from the current tile on the map to the goal tile.

## Path tracking
Pure pursuit is implemented to determine the steering angle for the rover, ensuring the path is followed with little deviance.

## Plugins
Plugins that were necessary:
- gz::sim::systems::Imu for simulated IMU.
- gz::sim::systems::DiffDrive for skid-steering.
- gz::sim::systems::Sensors for simulated lidar.
- gz::sim::systems::JointStatePublisher for publishing joint states.
- Everything else was implemented by hand.
