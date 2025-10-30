#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode() : Node("arm_controller_node")
  {
    // Declare parameter for selecting publisher type, default to "position"
    this->declare_parameter<std::string>("publisher_type", "position");
    publisher_type_ = this->get_parameter("publisher_type").as_string();

    // Subscriber to joint_states
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&ArmControllerNode::jointStateCallback, this, std::placeholders::_1));

    if (publisher_type_ == "position") {
      position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    } else if (publisher_type_ == "trajectory") {
      trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown publisher_type parameter, defaulting to position");
      position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    }

    // Timer to publish commands sequentially every 12 second
    timer_ = this->create_wall_timer(12s, std::bind(&ArmControllerNode::timerCallback, this));

    // Define four sample joint position commands
    position_commands_ = {
      {0.5, 0.0, 0.0, 0.0}, 
      {0.0, -0.5, 0.5, -1.0}, 
      {0.0, 0.0, 0.7, 0.0},  
      {-0.5, 0.0, 0.0, 1.0}
    };

    joint_names_ = {"j0", "j1", "j2", "j3"};
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Current joint positions:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
      RCLCPP_INFO(this->get_logger(), "  %s: %f", msg->name[i].c_str(), msg->position[i]);
    }
  }

  void timerCallback()
  {
    if (publisher_type_ == "position" && position_pub_) {
      std_msgs::msg::Float64MultiArray msg;
      msg.data = position_commands_[counter_ % position_commands_.size()];
      RCLCPP_INFO(this->get_logger(), "Publishing position command #%zu", counter_);
      position_pub_->publish(msg);
    } else if (publisher_type_ == "trajectory" && trajectory_pub_) {
      trajectory_msgs::msg::JointTrajectory traj_msg;
      traj_msg.joint_names = joint_names_;

      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = position_commands_[counter_ % position_commands_.size()];
      point.time_from_start = rclcpp::Duration::from_seconds(0.01); 
      point.velocities.resize(point.positions.size(), 10.0);
      traj_msg.points.push_back(point);

      RCLCPP_INFO(this->get_logger(), "Publishing trajectory command #%zu", counter_);
      trajectory_pub_->publish(traj_msg);
    }
    counter_++;
  }

  std::string publisher_type_;
  size_t counter_ = 0;
  std::vector<std::vector<double>> position_commands_;
  std::vector<std::string> joint_names_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

