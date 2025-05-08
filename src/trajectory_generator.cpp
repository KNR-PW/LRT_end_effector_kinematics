#include "trajectory_generator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

TrajectoryGenerator::TrajectoryGenerator()
: Node("trajectory_generator")
{
  declare_parameter<std::string>("robot_description", "");
  declare_parameter<double>("abduction_amplitude", 0.1);
  declare_parameter<double>("abduction_frequency", 0.5);
  declare_parameter<double>("circle_radius", 0.05);
  declare_parameter<double>("circle_frequency", 0.5);
  declare_parameter<double>("publish_frequency", 100.0);
  declare_parameter<double>("length_1", 0.225);
  declare_parameter<double>("length_2", 0.225);

  get_parameter("abduction_amplitude", ab_amp_);
  get_parameter("abduction_frequency", ab_freq_);
  get_parameter("circle_radius", circle_r_);
  get_parameter("circle_frequency", circle_freq_);
  get_parameter("publish_frequency", pub_freq_);
  get_parameter("length_1", l1_);
  get_parameter("length_2", l2_);
  if (pub_freq_ <= 0.0) pub_freq_ = 100.0;

  compute_centers_from_urdf();

  pub_ = create_publisher<geometry_msgs::msg::Vector3>(
    "end_effector_desired_trajectory", 10);

  sub_actual_ = create_subscription<geometry_msgs::msg::Vector3>(
    "end_effector_actual_trajectory", 10,
    std::bind(&TrajectoryGenerator::actual_callback, this, std::placeholders::_1));

  t0_ = now();
  auto period = std::chrono::duration<double>(1.0 / pub_freq_);
  timer_ = create_wall_timer(period,
    std::bind(&TrajectoryGenerator::on_timer, this));
}

void TrajectoryGenerator::actual_callback(
    const geometry_msgs::msg::Vector3::SharedPtr msg)
{
  actual_ = *msg;
}

void TrajectoryGenerator::compute_centers_from_urdf()
{
  std::string xml;
  get_parameter("robot_description", xml);
  model_.initString(xml);

  std::vector<std::string> chain_foot = {"M_joint","H_joint","K_joint","F_joint"};
  tf2::Transform Tfoot(tf2::Quaternion::getIdentity(), tf2::Vector3(0,0,0));
  for (auto& jn : chain_foot) {
    auto j = model_.getJoint(jn);
    if (!j) continue;
    const auto& o = j->parent_to_joint_origin_transform;
    Tfoot = Tfoot * tf2::Transform(
      tf2::Quaternion(o.rotation.x,o.rotation.y,o.rotation.z,o.rotation.w),
      tf2::Vector3(o.position.x,o.position.y,o.position.z));
  }
  tf2::Vector3 p_foot = Tfoot * tf2::Vector3(0,0,0);
  foot_center_ = {p_foot.x(), p_foot.y(), p_foot.z()};

  std::vector<std::string> chain_knee = {"M_joint","H_joint"};
  tf2::Transform Tk(tf2::Quaternion::getIdentity(), tf2::Vector3(0,0,0));
  for (auto& jn : chain_knee) {
    auto j = model_.getJoint(jn);
    if (!j) continue;
    const auto& o = j->parent_to_joint_origin_transform;
    Tk = Tk * tf2::Transform(
      tf2::Quaternion(o.rotation.x,o.rotation.y,o.rotation.z,o.rotation.w),
      tf2::Vector3(o.position.x,o.position.y,o.position.z));
  }
  if (auto jK = model_.getJoint("K_joint")) {
    const auto& oK = jK->parent_to_joint_origin_transform;
    Tk = Tk * tf2::Transform(
      tf2::Quaternion(oK.rotation.x,oK.rotation.y,oK.rotation.z,oK.rotation.w),
      tf2::Vector3(oK.position.x,oK.position.y,oK.position.z));
  }
  tf2::Vector3 p_knee = Tk * tf2::Vector3(0,0,0);
  knee_center_ = {p_knee.x(), p_knee.y(), p_knee.z()};
}

void TrajectoryGenerator::on_timer()
{
  double t = (now() - t0_).seconds();

  double ab_offset = ab_amp_ * std::sin(2*M_PI*ab_freq_*t);
  double y_knee = knee_center_.y + ab_offset;

  double angle = 2*M_PI * circle_freq_ * t;
  double x_des = knee_center_.x + circle_r_ * std::cos(angle);
  double z_des = knee_center_.z + circle_r_ * std::sin(angle);
  double y_des = y_knee;

  double dx = x_des - knee_center_.x;
  double dz = z_des - knee_center_.z;
  double r = std::hypot(dx, dz);
  double max_r = l1_ + l2_;
  if (r > max_r) {
    double s = max_r / r;
    dx *= s; dz *= s;
    x_des = knee_center_.x + dx;
    z_des = knee_center_.z + dz;
  }

  geometry_msgs::msg::Vector3 msg;
  msg.x = x_des;
  msg.y = y_des;
  msg.z = z_des;
  pub_->publish(msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryGenerator>());
  rclcpp::shutdown();
  return 0;
}