#include <memory>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <fstream>
#include <iostream>
#include <string>

struct TravelState {
  geometry_msgs::msg::Point point;
  float heading;
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

    goalPoint.x = 6;
    goalPoint.y = 8;
    goalPoint.z = 0.3;
    goalPose = 2.3;

    horizonHeading = -1;

    goalState.point.x = std::nanf("");
    goalState.point.y = std::nanf("");
    goalState.point.z = std::nanf("");
    goalState.heading = -1;

    // For plotting, we store data in a text file
    fout.open("/root/workspace/src/data.txt", std::ios::app);
  }
private:
  void publish_cmd_vel() 
  {
    // Based on applied ctrl algorithm
    publisher_->publish(ctrlSignal);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract position and velocity info from Odometry
    currPoint = msg->pose.pose.position;   
    currPose = msg->pose.pose;

    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;

    // Log the current position and velocity
    RCLCPP_INFO(this->get_logger(), "Odometry received: Position (%f, %f), Linear Velocity: %f, Angular Velocity: %f", 
         x, y, msg->twist.twist.linear.x, msg->twist.twist.angular.z);
 
    fout << x << " " << y << " ";

    computeOutput();
  }

 void quaternionToEuler(double &yaw, double &roll, double &pitch){
    tf2::Quaternion q(
        currPose.orientation.x,
        currPose.orientation.y,
        currPose.orientation.z,
        currPose.orientation.w
    );

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Current heading (yaw): %f radians", yaw);
  }

  float computeEuclideanDistance() {
    float delta_x_squared = std::pow(goalPoint.x - currPoint.x, 2);
    float delta_y_squared = std::pow(goalPoint.y - currPoint.y, 2);
    float summation = delta_x_squared + delta_y_squared;
    return std::sqrt(summation); 
  }

  float findPointDirection() {
    return std::atan2((goalPoint.y - currPoint.y), (goalPoint.x - currPoint.x));
  }

  void computeOutput() {
    // Compute eulerian orientation from quaternion
    double yaw, roll, pitch;

    // Determine euler angles
    quaternionToEuler(yaw, roll, pitch);

    fout << yaw << " ";

    double dirError = goalPose - yaw; 

    float distError = computeEuclideanDistance();
    
    horizonHeading = findPointDirection();

    fout << distError << std::endl;

    // Set initial pose - point not set
    if(goalState.heading == -1) {
      goalState.heading = horizonHeading;
    }
    // If state en route & at point - switch arrived
    else if(goalState.point.x != std::nanf("") && distError < 0.02) {
      goalState.heading = goalPose;
    }

    // If FindPoint - adjust angle to horizon heading
    if(goalState.point.x == std::nanf("")) {
      ctrlSignal.angular.z = kpDirection*(std::abs(yaw - horizonHeading));
      ctrlSignal.linear.x = 0; // do not move!
    }

    // If EnRoute - adjust speed to not overshoot the point
    else if(goalState.heading != goalPose) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Moving towards goal with %f to go", distError);
      ctrlSignal.linear.x = kpVelocity*distError;
      ctrlSignal.angular.z = kpDirection*(std::abs(yaw - horizonHeading));
    }

    // If arrived - adjust angle to final setpoint
    else {
      ctrlSignal.angular.z = kpDirection*(std::abs(dirError));
      ctrlSignal.linear.x = 0; // do not move!
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Computed output to be %f and %f", ctrlSignal.angular.z, ctrlSignal.linear.x);
  }

  std::ofstream fout; // for plotting data - columns are (x, y, yaw, euclidean distance)

  geometry_msgs::msg::Point currPoint;
  geometry_msgs::msg::Pose currPose;

  // direction to head to reach the point!
  float horizonHeading;

  const float kpDirection = 1.1;
  const float kpVelocity = 0.3;

  // control setpoint for desired state
  geometry_msgs::msg::Point goalPoint;
  float goalPose;

  // Output control signal
  geometry_msgs::msg::Twist ctrlSignal{};

  TravelState goalState; // determines what to adjust!

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