#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

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

  Eigen::Vector3d targetPosition;
  Eigen::Vector3d targetOrientationRpy;

  std::chrono::milliseconds iterationPeriod;
  std::size_t maxDemoIterations;
};

class R6BotIkDemoNode : public rclcpp::Node
{
public:
  R6BotIkDemoNode() : rclcpp::Node("r6bot_ik_demo_node"), demoIteration_(0), initialStatePublished_(false), demoFinished_(false)
  {
    settings_ = loadSettingsFromParameters();
    validateSettings();

    currentJointPositions_ = settings_.initialJointPositions;

    jointStatePublisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    markerPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/ik_demo_markers", 10);

    initializeKinematics();
    initializeTargetPose();

    timer_ = this->create_wall_timer(
      settings_.iterationPeriod,
      [this]()
      {
        if (!initialStatePublished_) {
          publishJointState();
          appendCurrentTcpPoint();
          publishTargetMarker();
          publishTcpTrailMarker();

          initialStatePublished_ = true;
          return;
        }

        runOneIkIteration();

        publishTargetMarker();
        publishTcpTrailMarker();
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
    settings.solverSettings.singularityThreshold = 1e-6;

    settings.solverName = this->declare_parameter<std::string>("solver_name", "NewtonRaphson");
    settings.solverSettings.stepCoefficient = this->declare_parameter<double>("step_coefficient", 0.2);

    if (settings.solverSettings.stepCoefficient <= 0.0 || settings.solverSettings.stepCoefficient > 1.0) {
      throw std::invalid_argument("step_coefficient must be greater than 0.0 and not greater than 1.0.");
    }

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

    const auto targetPosition = this->declare_parameter<std::vector<double>>(
      "target_position", {-0.535022568872121, 0.782303548336178, 1.21886466270994});

    const auto targetOrientationRpy = this->declare_parameter<std::vector<double>>(
      "target_orientation_rpy", {2.97925298910478, -1.12433897762375, 2.43916329232365});

    if (targetPosition.size() != 3) {
      throw std::invalid_argument("target_position must contain exactly 3 values: x, y, z.");
    }

    if (targetOrientationRpy.size() != 3) {
      throw std::invalid_argument("target_orientation_rpy must contain exactly 3 values: roll, pitch, yaw.");
    }

    settings.targetPosition = Eigen::Vector3d(targetPosition[0], targetPosition[1], targetPosition[2]);
    settings.targetOrientationRpy = Eigen::Vector3d(targetOrientationRpy[0], targetOrientationRpy[1], targetOrientationRpy[2]);

    settings.initialJointPositions = Eigen::Map<const Eigen::VectorXd>(
      initialJointPositions.data(), static_cast<Eigen::Index>(initialJointPositions.size()));

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

    if (!settings_.targetPosition.allFinite()) {
      throw std::invalid_argument("target_position must contain only finite values.");
    }

    if (!settings_.targetOrientationRpy.allFinite()) {
      throw std::invalid_argument("target_orientation_rpy must contain only finite values.");
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
    const double roll = settings_.targetOrientationRpy.x();
    const double pitch = settings_.targetOrientationRpy.y();
    const double yaw = settings_.targetOrientationRpy.z();

    const Eigen::Matrix3d targetRotation = (
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

    targetEndEffectorPoses_.assign(settings_.modelSettings.sixDofEndEffectorNames.size(), pinocchio::SE3::Identity());
    targetEndEffectorPoses_.front() = pinocchio::SE3(targetRotation, settings_.targetPosition);

    RCLCPP_INFO(this->get_logger(), "Target TCP position: [%.6f, %.6f, %.6f]",
      settings_.targetPosition.x(), settings_.targetPosition.y(), settings_.targetPosition.z());

    RCLCPP_INFO(this->get_logger(), "Target TCP RPY: [%.6f, %.6f, %.6f]", roll, pitch, yaw);
  }

  void appendCurrentTcpPoint()
  {
    std::vector<pinocchio::SE3> currentEndEffectorPoses(
      settings_.modelSettings.sixDofEndEffectorNames.size(), pinocchio::SE3::Identity());

    const auto fkStatus = kinematics_->calculateEndEffectorPoses(currentJointPositions_, currentEndEffectorPoses);

    if (!fkStatus.success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to calculate current TCP pose: %s", fkStatus.toString().c_str());
      return;
    }

    const auto & tcpPosition = currentEndEffectorPoses.front().translation();

    geometry_msgs::msg::Point point;
    point.x = tcpPosition.x();
    point.y = tcpPosition.y();
    point.z = tcpPosition.z();

    tcpTrailPoints_.push_back(point);
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

    appendCurrentTcpPoint();

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

  void publishTargetMarker()
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = settings_.modelSettings.baseLinkName;
    marker.header.stamp = this->now();

    marker.ns = "ik_target";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    const auto & targetPosition = targetEndEffectorPoses_.front().translation();

    marker.pose.position.x = targetPosition.x();
    marker.pose.position.y = targetPosition.y();
    marker.pose.position.z = targetPosition.z();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.6;

    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 0.8F;

    markerPublisher_->publish(marker);
  }

  void publishTcpTrailMarker()
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = settings_.modelSettings.baseLinkName;
    marker.header.stamp = this->now();

    marker.ns = "tcp_trail";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0F;
    marker.color.g = 0.4F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;

    marker.points = tcpTrailPoints_;

    markerPublisher_->publish(marker);
  }

  R6BotIkDemoSettings settings_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePublisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markerPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<multi_end_effector_kinematics::MultiEndEffectorKinematics> kinematics_;

  Eigen::VectorXd currentJointPositions_;
  std::vector<pinocchio::SE3> targetEndEffectorPoses_;
  std::vector<geometry_msgs::msg::Point> tcpTrailPoints_;

  std::size_t demoIteration_;
  bool initialStatePublished_;
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