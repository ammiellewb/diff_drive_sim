#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

using namespace std::chrono_literals;

class DiffDriveSim : public rclcpp::Node
{
    public:
        DiffDriveSim() : Node("diff_drive_sim") {
            // parameters
            this->declare_parameter("wheel_separation", 0.5);
            this->declare_parameter("wheel_radius", 0.1);
            this->declare_parameter("max_speed", 10.0);
            this->declare_parameter("max_steering", 1.0);

            wheel_separation_ = this->get_parameter("wheel_separation").as_double();
            wheel_radius_ = this->get_parameter("wheel_radius").as_double();
            max_speed_ = this->get_parameter("max_speed").as_double();
            max_steering_ = this->get_parameter("max_steering").as_double();

            // state variables (bicycle model)
            x_ = 0.0; // position x
            y_ = 0.0; // position y
            theta_ = 0.0; // orientation (heading angle)
            v_ = 0.0; // linear velocity
            omega_ = 0.0; // angular velocity (steering angle)

            // controller vars
            target_v_ = 0.0; // target linear velocity
            target_omega_ = 0.0; // target angular velocity (steering rate)

            // QoS settings
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

            // publishers
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
            pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);

            // subscriber
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", qos,
                std::bind(&DiffDriveSim::cmdVelCallback, this, std::placeholders::_1));
            
            // transform broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // timer
            timer_ = this->create_wall_timer(
                1ms, std::bind(&DiffDriveSim::updateSim, this));
            
            RCLCPP_INFO(this->get_logger(), "DiffDriveSim node started.");

        }

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
            // update target velocities, apply limits
            target_v_ = std::clamp(msg->linear.x, -max_speed_, max_speed_);
            target_omega_ = std::clamp(msg->angular.z, -max_steering_, max_steering_);
        }

        void updateSim() {
            auto now = this->now();

            // 1st order dynamics for velocity changes
            const double tau = 0.1; // time constant for velocity changes
            v_ += (target_v_ - v_) * (1.0-exp(tau));
            omega_ += (target_omega_ - omega_) * (1.0-exp(tau));

            // update state variables using bicycle model
            double dt = 0.02; // time step
            x_ += v_ * std::cos(theta_) * dt;
            y_ += v_ * std::sin(theta_) * dt;
            theta_ += omega_ * dt;   

            // wrap angle to [-pi, pi]
            if (theta_ > M_PI) {
                theta_ -= 2.0 * M_PI;
            } else if (theta_ < -M_PI) {
                theta_ += 2.0 * M_PI;
            }

            // publish odometry
            publishOdometry(now);

            // publish pose
            publishPose(now);

            // broadcast transform
            broadcastTransform(now);
        }

        void publishOdometry(const rclcpp::Time &now) {
            // publish odometry message
            auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
            odom_msg->header.stamp = now;
            odom_msg->header.frame_id = "odom";
            odom_msg->child_frame_id = "base_link";

            // set position and orientation
            odom_msg->pose.pose.position.x = x_;
            odom_msg->pose.pose.position.y = y_;
            odom_msg->pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            odom_msg->pose.pose.orientation.x = q.x();
            odom_msg->pose.pose.orientation.y = q.y();
            odom_msg->pose.pose.orientation.z = q.z();
            odom_msg->pose.pose.orientation.w = q.w();

            // set velocity
            odom_msg->twist.twist.linear.x = v_;
            odom_msg->twist.twist.linear.y = 0.0;
            odom_msg->twist.twist.linear.z = omega_;

            odom_pub_->publish(std::move(odom_msg));
        }

        void publishPose(const rclcpp::Time &now) {
            // publish pose message
            auto pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
            pose_msg->header.stamp = now;
            pose_msg->header.frame_id = "base_link";

            // set position and orientation
            pose_msg->pose.position.x = x_;
            pose_msg->pose.position.y = y_;
            pose_msg->pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            pose_msg->pose.orientation.x = q.x();
            pose_msg->pose.orientation.y = q.y();
            pose_msg->pose.orientation.z = q.z();
            pose_msg->pose.orientation.w = q.w();

            pose_pub_->publish(std::move(pose_msg));
        }
        void broadcastTransform(const rclcpp::Time &now) {
            // broadcast transform
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = now;
            transform.header.frame_id = "odom";
            transform.child_frame_id = "base_link";

            transform.transform.translation.x = x_;
            transform.transform.translation.y = y_;
            transform.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(transform);
        }

        // parameters
        double wheel_separation_;
        double wheel_radius_;
        double max_speed_;
        double max_steering_;

        // state variables
        double x_, y_, theta_;
        double v_, omega_;

        // target velocities
        double target_v_, target_omega_;
        
        // ROS2 components
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DiffDriveSim>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}