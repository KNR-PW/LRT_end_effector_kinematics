#include <end_effector_kinematics/QuadrupedBaseController.hpp>

QuadrupedBaseController::QuadrupedBaseController(pinocchio::Model& _model, pinocchio::Data& _data,
     std::vector<std::string> _feet_names): 
     Node("QuadrupedBaseController"), feet_names_{_feet_names},
    kinematics_solver(_model, _data, _feet_names), joint_names_{_model.names}
{
    joint_names_.erase(joint_names_.begin()); // Remove "Univserse" joint

    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
    
    twist_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",
     10, std::bind(&QuadrupedBaseController::control_callback, this, std::placeholders::_1));
    
    joint_configuration_ = Eigen::VectorXd(_model.njoints-1);
    joint_configuration_.setZero();
    joint_configuration_[1] = -M_PI/4;
    joint_configuration_[2] = M_PI/2;
    joint_configuration_[4] = -M_PI/4;
    joint_configuration_[5] = M_PI/2;
    joint_configuration_[7] = -M_PI/4;
    joint_configuration_[8] = M_PI/2;
    joint_configuration_[10] = -M_PI/4;
    joint_configuration_[11] = M_PI/2;

    joint_trajectories_.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint initial_trajectory_point;
    initial_trajectory_point.positions = std::vector<double>(_model.njoints-1, 0.0);
    joint_trajectories_.points.push_back(initial_trajectory_point);
    prepare_joint_trajectory_msg();
    joint_trajectories_.points[0].time_from_start.sec = 10;
    joint_trajectories_.points[0].time_from_start.nanosec = 0;
    trajectory_publisher_->publish(joint_trajectories_);

    kinematics_solver.computeCartesianConfiguration(joint_configuration_, feet_positions_);
}


void QuadrupedBaseController::control_callback(geometry_msgs::msg::Twist::UniquePtr _input_twist)
{
    input_twist_ = std::move(_input_twist);
    send_joint_trajectory_msg();
}

void QuadrupedBaseController::twist_to_joint_configuration()
{
    if(input_twist_ == nullptr) return;
    for(std::string feet_name: feet_names_)
    {
        pinocchio::SE3& feet_position = feet_positions_.at(feet_name);
        feet_position.translation().x() += 0.005 * input_twist_->linear.z;
        feet_position.translation().y() += 0.005 * input_twist_->linear.z;
        feet_position.translation().z() += 0.01 * input_twist_->linear.x;
    }
    kinematics_solver.computeJointConfiguration(joint_configuration_, feet_positions_);
    kinematics_solver.computeCartesianConfiguration(joint_configuration_, feet_positions_);
}

void QuadrupedBaseController::send_joint_trajectory_msg()
{
    twist_to_joint_configuration();
    prepare_joint_trajectory_msg();
    trajectory_publisher_->publish(joint_trajectories_);
}

void QuadrupedBaseController::prepare_joint_trajectory_msg()
{
    joint_trajectories_.header.stamp = this->now();
    auto& trajectory_point_msg = joint_trajectories_.points[0];
    std::memcpy(
                trajectory_point_msg.positions.data(), joint_configuration_.data(),
                trajectory_point_msg.positions.size() * sizeof(double));
    trajectory_point_msg.time_from_start.sec = 1;
    trajectory_point_msg.time_from_start.nanosec = 0; 
}
