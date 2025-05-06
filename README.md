# Differential Drive Simulator (Bicycle Model)

A ROS2 node that simulates a differential drive robot using the kinematic bicycle model, with controller outputs for odometry and pose.

## Features
- Simulates differential drive kinematics (bicycle model)
- Publishes odometry (`/odom`) and pose (`/pose`) data
- Supports velocity commands via `/cmd_vel`
- Includes TF transforms between `odom` and `base_link` frames

## Sources
- [Kinematic Bicycle Model Explained](https://www.shuffleai.blog/blog/Simple_Understanding_of_Kinematic_Bicycle_Model.html)
- [Bicycle Model Video Tutorial](https://www.youtube.com/watch?v=cjckvOHo8B4)
- [Algorithms for Automated Driving](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html)
- [ROS TF Quaternion Documentation](https://docs.ros.org/en/jade/api/tf/html/c++/classtf_1_1Quaternion.html)

## Installation & Usage

### Build the Package
```
colcon build --packages-select diff_drive_sim
source install/setup.bash
```
### Run the Node
```
ros2 run diff_drive_sim diff_drive_sim
```
### View Node Graph
```
rqt_graph
```
### Monitor Outputs
```
# View odometry data
ros2 topic echo /odom

# View pose data
ros2 topic echo /pose
```
### Test Commands
```
# Move Forward (0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1
# Rotate Counter-Clockwise (0.5 rad/s)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -1
# Move in a Curve (0.3 m/s forward + 0.2 rad/s turn)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}" -1
```
