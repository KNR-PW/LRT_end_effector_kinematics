#ifndef _QUADRUPED_BASE_CONTROLLER_HPP_
#define _QUADRUPED_BASE_CONTROLLER_HPP_

#include <end_effector_kinematics/MultiEndEffectorKinematics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <functional>
#include <math.h>

class QuadrupedBaseController: public rclcpp::Node
{
    private:

    trajectory_msgs::msg::JointTrajectory joint_trajectories_;
    geometry_msgs::msg::Twist::UniquePtr input_twist_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;

    std::vector<std::string> feet_names_;
    std::vector<std::string> joint_names_;
    Eigen::VectorXd joint_configuration_;
    std::unordered_map<std::string, pinocchio::SE3> feet_positions_;
    MultiEndEffectorKinematics kinematics_solver;

    void control_callback(geometry_msgs::msg::Twist::UniquePtr _input_twist);
    void twist_to_joint_configuration();
    void send_joint_trajectory_msg();
    void prepare_joint_trajectory_msg();

    public:

    QuadrupedBaseController(pinocchio::Model& _model, pinocchio::Data& _data,
     std::vector<std::string> _feet_names);

};

#endif