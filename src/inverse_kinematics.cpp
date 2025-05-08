#include "inverse_kinematics.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <algorithm>

InverseKinematics::InverseKinematics()
: Node("inverse_kinematics")
{
  loadParameters();
  computeCenterFromURDF();
  js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "end_effector_desired_trajectory", 10,
    std::bind(&InverseKinematics::traj_callback, this, std::placeholders::_1));
}

void InverseKinematics::loadParameters()
{
  declare_parameter<double>("length_1", 0.225);
  declare_parameter<double>("length_2", 0.225);
  get_parameter("length_1", l1_);
  get_parameter("length_2", l2_);
}

void InverseKinematics::computeCenterFromURDF()
{
  declare_parameter<std::string>("robot_description", "");
  std::string urdf_xml;
  get_parameter("robot_description", urdf_xml);
  model_.initString(urdf_xml);

  std::vector<std::string> chain = {"M_joint", "H_joint", "K_joint", "F_joint"};
  tf2::Transform T(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
  for (auto& jn : chain) {
    auto j = model_.getJoint(jn);
    const auto& o = j->parent_to_joint_origin_transform;
    tf2::Transform T0(
      tf2::Quaternion(o.rotation.x, o.rotation.y, o.rotation.z, o.rotation.w),
      tf2::Vector3(o.position.x, o.position.y, o.position.z));
    T = T * T0;
  }

  tf2::Vector3 p0 = T * tf2::Vector3(0, 0, 0);
  cx_ = p0.x();
  cy_ = p0.y();
  cz_ = p0.z();
}

void InverseKinematics::traj_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  double xr = msg->x - cx_;
  double yr = msg->y - cy_;
  double zr = msg->z - cz_;

  double q1 = std::atan2(yr, zr);
  double s1 = std::sin(q1);
  double c1 = std::cos(q1);

  double xr2 = xr;
  double zr2 = yr * s1 + zr * c1;

  double D = (xr2 * xr2 + zr2 * zr2 - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
  D = std::clamp(D, -1.0, 1.0);

  double q3 = -std::acos(D);
  double alpha = std::atan2(zr2, xr2);
  double beta = std::atan2(l2_ * std::sin(q3), l1_ + l2_ * std::cos(q3));
  double q2 = alpha - beta;

  sensor_msgs::msg::JointState js;
  js.header.stamp = now();
  js.name = {"M_joint", "H_joint", "K_joint"};
  js.position = {q1, q2, q3};
  js_pub_->publish(js);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InverseKinematics>());
  rclcpp::shutdown();
  return 0;
}
