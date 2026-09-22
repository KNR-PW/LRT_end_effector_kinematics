// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/spatial/se3.hpp>

#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>
#include <multi_end_effector_kinematics/Settings.hpp>

namespace
{
    constexpr double kValidationTolerance = 1e-4;

    std::string getUrdfPath(int argc, char* argv[])
    {
        if (argc > 2) {
            throw std::invalid_argument("Usage: BasicInverseKinematics [path/to/r6bot.urdf]");
        }

        if (argc == 2) {
            return argv[1];
        }

        const auto packageShare = ament_index_cpp::get_package_share_directory("multi_end_effector_kinematics");

        return packageShare + "/models/r6bot/r6bot.urdf";
    }

    pinocchio::SE3 createTargetPose()
    {
        const Eigen::Vector3d targetPosition(-0.535022568872121, 0.782303548336178, 1.21886466270994);

        const double roll = 2.97925298910478;
        const double pitch = -1.12433897762375;
        const double yaw = 2.43916329232365;

        const Eigen::Matrix3d targetRotation =
            (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();

        return pinocchio::SE3(targetRotation, targetPosition);
    }
} 

int main(int argc, char* argv[])
{
  using namespace multi_end_effector_kinematics;

  try {
    KinematicsModelSettings modelSettings;
    modelSettings.baseLinkName = "world";
    modelSettings.sixDofEndEffectorNames = {"tool0"};

    InverseSolverSettings solverSettings;
    solverSettings.maxIterations = 1000;
    solverSettings.tolerance = 1e-5;
    solverSettings.minimumStepSize = 1e-8;
    solverSettings.dampingCoefficient = 1e-6;
    solverSettings.stepCoefficient = 0.2;
    solverSettings.singularityThreshold = 1e-6;

    MultiEndEffectorKinematics kinematics(getUrdfPath(argc, argv), modelSettings, solverSettings, "NewtonRaphson");

    const Eigen::VectorXd initialJointPositions = Eigen::VectorXd::Zero(kinematics.getPinocchioModel().nq);

    const std::vector<pinocchio::SE3> targetPoses{ createTargetPose() };

    Eigen::VectorXd solvedJointPositions;

    const ReturnStatus ikStatus = kinematics.calculateJointPositions(initialJointPositions, targetPoses, solvedJointPositions);

    if (!ikStatus.success || ikStatus.flag != TaskReturnFlag::FINISHED) {
      std::cerr << "Inverse kinematics failed: " << ikStatus.toString() << '\n';
      return 1;
    }

    std::vector<pinocchio::SE3> achievedPoses(1, pinocchio::SE3::Identity());

    const ReturnStatus fkStatus = kinematics.calculateEndEffectorPoses(solvedJointPositions, achievedPoses);

    if (!fkStatus.success) {
      std::cerr << "Forward kinematics validation failed: " << fkStatus.toString() << '\n';
      return 1;
    }

    const double positionError = (achievedPoses.front().translation() - targetPoses.front().translation()).norm();

    const double rotationError = (achievedPoses.front().rotation() - targetPoses.front().rotation()).norm();

    std::cout << "IK finished in " << ikStatus.iterations << " iterations.\n";

    std::cout << "Joint solution: " << solvedJointPositions.transpose() << '\n';

    std::cout << "Position error: " << positionError << '\n';

    std::cout << "Rotation error: " << rotationError << '\n';

    if (positionError > kValidationTolerance || rotationError > kValidationTolerance) {
      std::cerr << "The calculated pose does not match the requested target.\n";
      return 1;
    }

    return 0;
  } 
  catch (const std::exception& exception) {
    std::cerr << "Example failed: " << exception.what() << '\n';
    return 1;
  }
}