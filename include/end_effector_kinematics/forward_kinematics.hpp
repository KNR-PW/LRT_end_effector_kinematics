#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP

#include <vector>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"
#include "tf2/LinearMath/Transform.h"

class ForwardKinematics : public rclcpp::Node
{
public:
  ForwardKinematics();

private:
  bool loadModel();
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void computeFK(const std::map<std::string, double>& q);

  urdf::Model model_;
  std::vector<std::string> chain_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

#endif // FORWARD_KINEMATICS_HPP
