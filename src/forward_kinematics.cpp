#include "forward_kinematics.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <algorithm>
#include "tf2/LinearMath/Transform.h"

ForwardKinematics::ForwardKinematics()
: Node("forward_kinematics"), chain_{"M_joint", "H_joint", "K_joint"}
{
  loadModel();
  pub_ = create_publisher<geometry_msgs::msg::Vector3>("end_effector_actual_trajectory", 10);
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&ForwardKinematics::joint_state_callback, this, std::placeholders::_1));
}

bool ForwardKinematics::loadModel()
{
  declare_parameter<std::string>("robot_description", "");
  std::string urdf_xml;
  get_parameter("robot_description", urdf_xml);
  return model_.initString(urdf_xml);
}

void ForwardKinematics::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::map<std::string, double> q;
  for (size_t i = 0; i < msg->name.size(); ++i)
    if (std::find(chain_.begin(), chain_.end(), msg->name[i]) != chain_.end())
      q[msg->name[i]] = msg->position[i];
  if (q.size() == chain_.size())
    computeFK(q);
}

void ForwardKinematics::computeFK(const std::map<std::string, double>& q)
{
  tf2::Transform T(tf2::Quaternion::getIdentity(), tf2::Vector3(0, 0, 0));
  for (auto& jn : chain_) {
    auto joint = model_.getJoint(jn);
    const auto& p = joint->parent_to_joint_origin_transform.position;
    const auto& r = joint->parent_to_joint_origin_transform.rotation;
    tf2::Transform T_origin(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(p.x, p.y, p.z));
    tf2::Quaternion q_rot;
    q_rot.setRotation(tf2::Vector3(joint->axis.x, joint->axis.y, joint->axis.z), q.at(jn));
    T = T * T_origin * tf2::Transform(q_rot, tf2::Vector3(0, 0, 0));
  }
  if (auto j_fixed = model_.getJoint("F_joint")) {
    const auto& p = j_fixed->parent_to_joint_origin_transform.position;
    const auto& r = j_fixed->parent_to_joint_origin_transform.rotation;
    T = T * tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(p.x, p.y, p.z));
  }
  tf2::Vector3 foot = T * tf2::Vector3(0, 0, 0);
  geometry_msgs::msg::Vector3 out;
  out.x = foot.x();
  out.y = foot.y();
  out.z = foot.z();
  pub_->publish(out);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForwardKinematics>());
  rclcpp::shutdown();
  return 0;
}
