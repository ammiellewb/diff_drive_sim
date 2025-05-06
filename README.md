# Differential Drive Simulator (Bicycle Model)

A ROS2 node that simulates a differential drive robot using the kinematic bicycle model, with controller outputs for odometry and pose.

## Key Features
- Simulates differential drive kinematics (bicycle model)
- Publishes odometry (`/odom`) and pose (`/pose`) data
- Supports velocity commands via `/cmd_vel`
- Includes TF transforms between `odom` and `base_link` frames

## Theory Resources
- [Kinematic Bicycle Model Explained](https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html)
- [Bicycle Model Video Tutorial](https://www.youtube.com/watch?v=cjckvOHo8B4)
- [Algorithms for Automated Driving](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [ROS TF Quaternion Documentation](https://docs.ros.org/en/jade/api/tf/html/c++/classtf_1_1Quaternion.html)

## Installation & Usage

### Build the Package
```
colcon build --packages-select diff_drive_sim
source install/setup.bash
ros2 run diff_drive_sim diff_drive_sim
```
