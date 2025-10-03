// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

#ifndef __SETTINGS_MULTI_END_EFFECTOR_KINEMATICS__
#define __SETTINGS_MULTI_END_EFFECTOR_KINEMATICS__


#include <string>
#include <vector>

namespace multi_end_effector_kinematics
{
  struct InverseSolverSettings
  {
    // Max iterations for solving inverse kinematics
    unsigned maxIterations = 100;

    // Task error tolerance
    double tolerance = 1e-6;

    // Minimum step size between iteration
    double minimumStepSize = 1e-6;

    // Damping coefficient for damped inverse jacobian matrix: J^T * (J * J^T + λ * I)^-1
    double dampingCoefficient = 1e-10; 

    // Coefficient multiplied with new Δq
    double stepCoefficient = 0.5; // > 0

    // Threshold for checking if robot is in singularity
    double singularityThreshold = 1e-6;
  };

  struct KinematicsModelSettings
  {
    // Base link (frame that forward and inverse kinematics are defined)
    std::string baseLinkName;

    // Vector of all end effectors with 3 DoF
    std::vector<std::string> threeDofEndEffectorNames;

    // Vector of all end effectors with 6 DoF
    std::vector<std::string> sixDofEndEffectorNames;
  };

  struct KinematicsInternalModelSettings
  {
    // Base frame index (frame that forward and inverse kinematics are defined)
    size_t baseFrameIndex;
    
    // Number of all end effectors
    size_t numEndEffectors;

    // 3 DoF end effectors, position only                  
    size_t numThreeDofEndEffectors;
    
    // 6 DoF end effectors, position and orientation
    size_t numSixDofEndEffectors;

    // Indices of end-effector frames [3DOF end effectors, 6DOF end effectors]   
    std::vector<size_t> endEffectorFrameIndices;

    // Indices of end-effector parent joints [3DOF end effectors, 6DOF end effectors]
    std::vector<size_t> endEffectorJointIndices;
  };


  enum class TaskReturnFlag
  {
    FINISHED = 0, // true
    IN_PROGRESS = 1,

    SOLVER_ERROR = 2, //  false
    CURRENT_POSITION_OUT_OF_BOUNDS = 3,
    CURRENT_VELOCITY_OUT_OF_BOUNDS = 4,
    NEW_POSITION_OUT_OF_BOUNDS = 5,
    NEW_VELOCITY_OUT_OF_BOUNDS = 6,
    SMALL_STEP_SIZE = 7,
  };

  enum class InverseSolverType
  {
    GRADIENT_BASED = 0,
    QP_BASED = 1,
  };

  enum class TaskType
  {
    NORMAL = 0,
    REDUNDANT = 1,
    DAMPED = 2,
  };

  struct ReturnStatus
  {
    bool success;
    TaskReturnFlag flag;

    std::string toString() const;
  };  

  // Helper functions
  std::string returnFlagToString(TaskReturnFlag flag);
  std::string solverTypeToString(InverseSolverType solverType);
  std::string taskTypeToString(TaskType taskType);

}; // namespace multi_end_effector_kinematics

#endif