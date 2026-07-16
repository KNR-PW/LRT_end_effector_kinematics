#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pinocchio/spatial/se3.hpp>

#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>
#include <multi_end_effector_kinematics/Settings.hpp>

struct R6BotIkDemoSettings
{
  multi_end_effector_kinematics::KinematicsModelSettings modelSettings;
  multi_end_effector_kinematics::InverseSolverSettings solverSettings;

  std::string solverName;

  std::vector<std::string> jointNames;

  Eigen::VectorXd initialJointPositions;
  Eigen::VectorXd goalJointPositions;

  std::chrono::milliseconds iterationPeriod;
  std::size_t maxDemoIterations;
};

class R6BotIkDemoNode : public rclcpp::Node
{
public:
  R6BotIkDemoNode() : rclcpp::Node("r6bot_ik_demo_node"), demoIteration_(0), demoFinished_(false)
  {
    settings_ = loadSettingsFromParameters();
    validateSettings();

    currentJointPositions_ = settings_.initialJointPositions;

    jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    initializeKinematics();
    initializeTargetPose();

    timer_ = this->create_wall_timer(settings_.iterationPeriod,
      [this]()
      {
        this->runOneIkIteration();
      });

    RCLCPP_INFO(this->get_logger(), "R6Bot IK demo node started.");
    RCLCPP_INFO(this->get_logger(), "Solver: %s", settings_.solverName.c_str());
    RCLCPP_INFO(this->get_logger(), "Iteration period: %ld ms", settings_.iterationPeriod.count());
  }

private:
  R6BotIkDemoSettings loadSettingsFromParameters()
  {
    R6BotIkDemoSettings settings;

    settings.modelSettings.baseLinkName = "world";
    settings.modelSettings.threeDofEndEffectorNames = {};
    settings.modelSettings.sixDofEndEffectorNames = {"tool0"};

    settings.solverSettings.maxIterations = 1000;
    settings.solverSettings.tolerance = 1e-5;
    settings.solverSettings.minimumStepSize = 1e-8;
    settings.solverSettings.dampingCoefficient = 1e-6;
    settings.solverSettings.stepCoefficient = 0.8;
    settings.solverSettings.singularityThreshold = 1e-6;

    settings.solverName = this->declare_parameter<std::string>("solver_name", "NewtonRaphson");

    settings.jointNames = {
      "joint_1",
      "joint_2",
      "joint_3",
      "joint_4",
      "joint_5",
      "joint_6"
    };

    const auto initialJointPositions = this->declare_parameter<std::vector<double>>(
        "initial_joint_positions", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    const auto goalJointPositions = this->declare_parameter<std::vector<double>>(
        "goal_joint_positions", {0.30, -0.45, 0.55, 0.25, -0.35, 0.15});

    settings.initialJointPositions = Eigen::Map<const Eigen::VectorXd>(initialJointPositions.data(), static_cast<Eigen::Index>(initialJointPositions.size()));

    settings.goalJointPositions =
      Eigen::Map<const Eigen::VectorXd>(goalJointPositions.data(), static_cast<Eigen::Index>(goalJointPositions.size()));

    const int iterationPeriodMs = this->declare_parameter<int>("iteration_period_ms", 100);

    if (iterationPeriodMs <= 0) {
      throw std::invalid_argument("iteration_period_ms must be greater than 0.");
    }

    settings.iterationPeriod = std::chrono::milliseconds(iterationPeriodMs);

    const int maxDemoIterations = this->declare_parameter<int>("max_demo_iterations", 200);

    if (maxDemoIterations <= 0) {
      throw std::invalid_argument("max_demo_iterations must be greater than 0.");
    }

    settings.maxDemoIterations = static_cast<std::size_t>(maxDemoIterations);

    return settings;
  }

  void validateSettings() const
  {
    const auto expectedNumberOfJoints = settings_.jointNames.size();

    if (expectedNumberOfJoints == 0) {
      throw std::invalid_argument("jointNames must not be empty.");
    }

    if (static_cast<std::size_t>(settings_.initialJointPositions.size()) != expectedNumberOfJoints) {
      throw std::invalid_argument("initial_joint_positions size must match the number of r6bot joints.");
    }

    if (static_cast<std::size_t>(settings_.goalJointPositions.size()) != expectedNumberOfJoints) {
      throw std::invalid_argument("goal_joint_positions size must match the number of r6bot joints.");
    }

    if (settings_.solverName.empty()) {
      throw std::invalid_argument("solver_name must not be empty.");
    }

    if (settings_.modelSettings.baseLinkName.empty()) {
      throw std::invalid_argument("modelSettings.baseLinkName must not be empty.");
    }

    if (settings_.modelSettings.sixDofEndEffectorNames.empty() || settings_.modelSettings.sixDofEndEffectorNames.front().empty()) {
      throw std::invalid_argument("modelSettings.sixDofEndEffectorNames must contain one valid end-effector name.");
    }
  }

  void initializeKinematics()
  {
    kinematics_ = std::make_unique<multi_end_effector_kinematics::MultiEndEffectorKinematics>(
        getR6BotUrdfPath(), settings_.modelSettings, settings_.solverSettings, settings_.solverName);

    RCLCPP_INFO(this->get_logger(), "Kinematics object initialized.");
  }

  void initializeTargetPose()
  {
    targetEndEffectorPoses_.assign(settings_.modelSettings.sixDofEndEffectorNames.size(), pinocchio::SE3::Identity());

    const auto fkStatus = kinematics_->calculateEndEffectorPoses(settings_.goalJointPositions, targetEndEffectorPoses_);

    if (!fkStatus.success) {
      throw std::runtime_error("Failed to compute FK for goal_joint_positions: " + fkStatus.toString());
    }

    RCLCPP_INFO(this->get_logger(), "Target end-effector pose initialized from FK(goal_joint_positions).");
  }

  std::string getR6BotUrdfPath() const
  {
    const auto packageShare = ament_index_cpp::get_package_share_directory("multi_end_effector_kinematics");

    return packageShare + "/models/r6bot/r6bot.urdf";
  }

  void runOneIkIteration()
  {
    if (demoFinished_) {
      publishJointState();
      return;
    }

    Eigen::VectorXd jointDelta = Eigen::VectorXd::Zero(currentJointPositions_.size());

    const auto ikStatus = kinematics_->calculateJointDeltas(currentJointPositions_, targetEndEffectorPoses_, jointDelta);

    if (!ikStatus.success) {
      RCLCPP_ERROR(this->get_logger(), "IK iteration failed: %s", ikStatus.toString().c_str());

      demoFinished_ = true;
      publishJointState();
      return;
    }

    if (ikStatus.flag == multi_end_effector_kinematics::TaskReturnFlag::FINISHED) {
      RCLCPP_INFO(this->get_logger(), "IK target reached after %zu demo iterations.", demoIteration_);

      demoFinished_ = true;
      publishJointState();
      return;
    }

    if (jointDelta.size() != currentJointPositions_.size()) {
      RCLCPP_ERROR(this->get_logger(), "IK returned jointDelta of invalid size: %ld", jointDelta.size());

      demoFinished_ = true;
      publishJointState();
      return;
    }

    currentJointPositions_ += jointDelta;
    ++demoIteration_;

    if (demoIteration_ >= settings_.maxDemoIterations) {
      RCLCPP_WARN(this->get_logger(), "IK demo reached max_demo_iterations=%zu.", settings_.maxDemoIterations);

      demoFinished_ = true;
    }

    publishJointState();
  }

  void publishJointState()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = settings_.jointNames;
    msg.position.resize(static_cast<std::size_t>(currentJointPositions_.size()));

    for (int i = 0; i < currentJointPositions_.size(); ++i) {
      msg.position[static_cast<std::size_t>(i)] = currentJointPositions_[i];
    }

    jointStatePublisher_->publish(msg);
  }

  R6BotIkDemoSettings settings_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<multi_end_effector_kinematics::MultiEndEffectorKinematics> kinematics_;

  Eigen::VectorXd currentJointPositions_;
  std::vector<pinocchio::SE3> targetEndEffectorPoses_;

  std::size_t demoIteration_;
  bool demoFinished_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<R6BotIkDemoNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}