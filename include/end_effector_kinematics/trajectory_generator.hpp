#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "urdf/model.h"


class TrajectoryGenerator : public rclcpp::Node
{
public:
  TrajectoryGenerator();

private:
  void compute_centers_from_urdf();
  void on_timer();
  void actual_callback(const geometry_msgs::msg::Vector3::SharedPtr);

  double ab_amp_{}, ab_freq_{}, circle_r_{}, circle_freq_{}, pub_freq_{};
  double l1_{}, l2_{};
  struct Center { double x{}, y{}, z{}; };
  Center foot_center_{}, knee_center_{};
  geometry_msgs::msg::Vector3 actual_{};
  rclcpp::Time t0_{};
  urdf::Model model_{};
  rclcpp::TimerBase::SharedPtr timer_{};
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_{};
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_actual_{};
};

#endif