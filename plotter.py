#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import time

class DiffDriveStatePlotter(Node):
    def __init__(self):
        super().__init__('plotter')
        
        # data collection
        self.times = []
        self.positions_x = []
        self.positions_y = []
        self.orientations = []
        self.velocities_linear = []
        self.velocities_angular = []
        
        # starting timestamp
        self.start_time = None
        self.is_collecting = False
        
        # publisher for velocity commands
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel',
            10
        )
        
        # subscriber for odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        # debug print for troubleshooting
        # self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('Differential Drive State Plotter initialized')

    def print_status(self):
        self.get_logger().info(f'Data points collected: {len(self.times)}')
    
    def odom_callback(self, msg):
        if not self.is_collecting:
            return

        # initialize start time if this is the first message
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('First odometry message received')
        
        # elapsed time
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        
        # pose information
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        
        # convert quaternion to yaw angle (theta)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # convert quaternion to Euler angles (yaw)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # velocity information
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # store data
        self.times.append(elapsed_time)
        self.positions_x.append(position_x)
        self.positions_y.append(position_y)
        self.orientations.append(yaw)
        self.velocities_linear.append(linear_vel)
        self.velocities_angular.append(angular_vel)

        if len(self.times) % 20 == 0:
            self.get_logger().info(f'Data points collected: {len(self.times)}')
            self.get_logger().info(f'Latest position: ({position_x:.3f}, {position_y:.3f}), '
                              f'v={linear_vel:.3f}, omega={angular_vel:.3f}')
    
    def send_constant_velocity(self, linear_vel, angular_vel, duration):
        # send a constant velocity command for a specified duration
        msg = Twist()
        msg.linear.x = float(linear_vel)
        msg.angular.z = float(angular_vel)
        
        
        # reset data collection
        self.times = []
        self.positions_x = []
        self.positions_y = []
        self.orientations = []
        self.velocities_linear = []
        self.velocities_angular = []
        self.start_time = None

        # start collecting data
        self.is_collecting = True
        self.get_logger().info(f'Starting test: linear={linear_vel}, angular={angular_vel}, duration={duration}s')
        
        # publish command
        self.cmd_vel_publisher.publish(msg)
        
        # wait while processing callbacks
        start = time.time()
        while time.time() - start < duration:
            time.sleep(0.1)  # sleep briefly - ROS callbacks run in separate threads
        
        # stop
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        self.is_collecting = False
        time.sleep(0.5)  # Wait for final messages
        
        # log results
        self.get_logger().info(f'Test complete. Collected {len(self.times)} data points')
        
        # plot results
        if len(self.times) > 0:
            self.plot_results(f"Linear: {linear_vel}, Angular: {angular_vel}")
        else:
            self.get_logger().error('No data collected')
    
    def plot_results(self, title_suffix=""):
        """Plot the collected state data"""
        if len(self.times) == 0:
            self.get_logger().error('No data collected')
            return
        
        self.get_logger().info(f'Plotting data with {len(self.times)} points')
        
        # Create a figure with subplots
        fig, axs = plt.subplots(3, 2, figsize=(12, 10))
        fig.suptitle(f"Differential Drive Simulation - {title_suffix}", fontsize=16)
        
        # Position vs time (x and y)
        axs[0, 0].plot(self.times, self.positions_x, 'b-', label='X Position')
        axs[0, 0].plot(self.times, self.positions_y, 'r-', label='Y Position')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Position (m)')
        axs[0, 0].set_title('Position vs Time')
        axs[0, 0].legend()
        axs[0, 0].grid(True)
        
        # Trajectory (y vs x)
        axs[0, 1].plot(self.positions_x, self.positions_y, 'g-')
        axs[0, 1].set_xlabel('X Position (m)')
        axs[0, 1].set_ylabel('Y Position (m)')
        axs[0, 1].set_title('Trajectory')
        axs[0, 1].grid(True)
        # Make square aspect ratio
        axs[0, 1].set_aspect('equal')
        
        # Orientation vs time
        axs[1, 0].plot(self.times, self.orientations, 'b-')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Orientation (rad)')
        axs[1, 0].set_title('Orientation vs Time')
        axs[1, 0].grid(True)
        
        # Linear velocity vs time
        axs[1, 1].plot(self.times, self.velocities_linear, 'r-')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Linear Velocity (m/s)')
        axs[1, 1].set_title('Linear Velocity vs Time')
        axs[1, 1].grid(True)
        
        # Angular velocity vs time
        axs[2, 0].plot(self.times, self.velocities_angular, 'g-')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Angular Velocity (rad/s)')
        axs[2, 0].set_title('Angular Velocity vs Time')
        axs[2, 0].grid(True)
        
        # Robot pose at various time points
        axs[2, 1].plot(self.positions_x, self.positions_y, 'g-', alpha=0.5)
        
        # Orientation markers
        num_markers = min(10, len(self.times))
        if num_markers > 0:
            indices = np.linspace(0, len(self.times)-1, num_markers, dtype=int)
            for i in indices:
                x = self.positions_x[i]
                y = self.positions_y[i]
                theta = self.orientations[i]
                
                # draw arrow to represent robot orientation
                arrow_length = 0.2
                dx = arrow_length * math.cos(theta)
                dy = arrow_length * math.sin(theta)
                
                axs[2, 1].arrow(x, y, dx, dy, head_width=0.05, head_length=0.08, fc='blue', ec='blue')
        
        axs[2, 1].set_xlabel('X Position (m)')
        axs[2, 1].set_ylabel('Y Position (m)')
        axs[2, 1].set_title('Robot Pose Visualization')
        axs[2, 1].grid(True)
        axs[2, 1].set_aspect('equal')
        
        # adjust layout
        plt.tight_layout()
        plt.savefig(f"graphs/diff_drive_sim_{title_suffix.replace(' ', '_').replace(':', '')}.png")
        plt.show()

def spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    plotter = DiffDriveStatePlotter()

    thread = threading.Thread(target=spin_thread, args=(plotter,))
    thread.daemon = True
    thread.start()
    
    # test cases to demonstrate controller behavior
    try:
        time.sleep(2)  # time for the node to initialize

        # test case 1: Move forward
        plotter.send_constant_velocity(0.5, 0.0, 10.0)  # 0.5 m/s forward, 5 seconds
        
        # test case 2: Pure rotation
        plotter.send_constant_velocity(0.0, 0.5, 10.0)  # 0.5 rad/s rotation, 5 seconds
        
        # test case 3: Curved path
        plotter.send_constant_velocity(0.3, 0.2, 15.0)  # 0.3 m/s forward, 0.2 rad/s rotation, 10 seconds
        
        # test case 4: S-curve (positive then negative angular velocity)
        plotter.send_constant_velocity(0.3, 0.3, 10.0)  # First curve
        plotter.send_constant_velocity(0.3, -0.3, 10.0)  # Second curve
        
    except KeyboardInterrupt:
        pass
        
    rclpy.shutdown()
    if thread.is_alive():
        thread.join(timeout=3.0)

if __name__ == '__main__':
    main()