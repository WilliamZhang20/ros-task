#include <memory>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

enum class TravelState {
    FindPoint,
    EnRoute,
    Arrived
};

class RobotControlNode : public rclcpp::Node
{
public:
  RobotControlNode()
  : Node("RobotControlNode")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&RobotControlNode::odom_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&RobotControlNode::publish_cmd_vel, this));

    goalPoint.x = 10;
    goalPoint.y = 10;
    goalPoint.z = 0;
    goalPose = 2.3;

    goal = TravelState::FindPoint;
  }
private:
  void publish_cmd_vel() 
  {
    geometry_msgs::msg::Twist ctrlSignal;

    // Based on applied ctrl algorithm
    ctrlSignal.linear.x = 1.0;  // Forward speed in m/s
    ctrlSignal.angular.z = 0.5; // Rotation speed in rad/s

    publisher_->publish(ctrlSignal);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract position and velocity info from Odometry
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    linear_vel = msg->twist.twist.linear.x;
    angular_vel = msg->twist.twist.angular.z;

    // Log the current position and velocity
    RCLCPP_INFO(this->get_logger(), "Odometry received: Position (%f, %f), Linear Velocity: %f, Angular Velocity: %f",
                x, y, linear_vel, angular_vel);

    computeOutput();
  }


 void quaternionToeuler(const geometry_msgs::msg::Pose::SharedPtr quatPose, double &yaw, double &roll, double &pitch){
    tf2::Quaternion q(
        quatPose->orientation.x,
        quatPose->orientation.y,
        quatPose->orientation.z,
        quatPose->orientation.w
    );

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current heading (yaw): %f radians", yaw);
  };

  void computeOutput() {
    // Compute eulerian orientation from quaternion

    // If state FindPoint & NaN horizonHeading - determine horizonHeading 

    // If state FindPoint & are lined up with horizonHeading - switch to en route

    // If state en route & at point - switch arrived

    // If FindPoint - adjust angle to vector between
    // If EnRoute - adjust speed
    // If arrived - adjust angle to final setpoint
  }

  float dirOutput = 0.0f; 
  float pwrOutput = 0.0f;

  float x; float y;
  float linear_vel; float angular_vel;

  // direction to head to reach the point!
  float horizonHeading = std::nanf("");

  const float kpDirection = 1.5;
  const float kpVelocity = 1.5;

  // control setpoint for desired state
  geometry_msgs::msg::Point goalPoint;
  float goalPose;

  TravelState goal; // determines what to adjust!

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotControlNode>());
  rclcpp::shutdown();
  return 0;
}