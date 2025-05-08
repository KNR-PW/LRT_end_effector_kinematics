#ifndef INVERSE_KINEMATICS_HPP
#define INVERSE_KINEMATICS_HPP

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "urdf/model.h"


class InverseKinematics : public rclcpp::Node
{
public:
  InverseKinematics();

private:
  void loadParameters();
  void computeCenterFromURDF();
  void traj_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

  double l1_{0.225}, l2_{0.225};
  double cx_{0.0}, cy_{0.0}, cz_{0.0};
  urdf::Model model_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
};

#endif // INVERSE_KINEMATICS_HPP
