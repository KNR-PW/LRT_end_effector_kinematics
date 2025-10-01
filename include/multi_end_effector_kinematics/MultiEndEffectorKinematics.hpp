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

#ifndef __MULTI_END_EFFECTOR_KINEMATICS_MULTI_END_EFFECTOR_KINEMATICS__
#define __MULTI_END_EFFECTOR_KINEMATICS_MULTI_END_EFFECTOR_KINEMATICS__

#include <utility>
#include <memory>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <multi_end_effector_kinematics/Settings.hpp>
#include <multi_end_effector_kinematics/solvers/InverseSolverInterface.hpp>
#include <multi_end_effector_kinematics/solvers/SolversList.hpp>

namespace multi_end_effector_kinematics
{
  class MultiEndEffectorKinematics
  {

    public:

      /**
       * Constructor
       * Create forward and inverse kinematics solver for multiple end effectors
       * @warning Everything in input and output is defined in base frame of reference!
       *
       * @param [in] urdfFilePath: Absolute file path to URDF file
       * @param [in] modelSettings: Settings for kinematic model
       * @param [in] solverSettings: Settings for inverse kinematics solver
       * @param [in] solverName: name of solver (currently only "NewtonRaphson")
       */
      MultiEndEffectorKinematics(const std::string urdfFilePath,
        const KinematicsModelSettings modelSettings,
        const InverseSolverSettings solverSettings,
        const std::string solverName);
      
      /**
       * Get one iteration of inverse kinematics algorithm (Δq) of q[n+1] = q[n] + Δq
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [in] endEffectorTransforms: Transforms of 6D end effectors
       * @param [out] jointDeltas: joint deltas Δq
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        const std::vector<pinocchio::SE3>& endEffectorTransforms,
        Eigen::VectorXd& jointDeltas);
      
      /**
       * Get one iteration of inverse kinematics algorithm (Δq) of q[n+1] = q[n] + Δq
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [out] jointDeltas: joint deltas Δq
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        Eigen::VectorXd& jointDeltas);
      
      /**
       * Get one iteration of inverse kinematics algorithm (Δq) of q[n+1] = q[n] + Δq
       * Version for models with only 6D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorTransforms: Transforms of 6D end effectors
       * @param [out] jointDeltas: joint deltas Δq
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<pinocchio::SE3>& endEffectorTransforms,
        Eigen::VectorXd& jointDeltas);
      
      /**
       * Get joint positions from inverse kinematics 
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [in] endEffectorTransforms: Transforms of 6D end effectors
       * @param [out] newJointPositions: new joint positions
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        const std::vector<pinocchio::SE3>& endEffectorTransforms,
        Eigen::VectorXd& newJointPositions);
      
      /**
       * Get joint positions from inverse kinematics
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [out] newJointPositions: new joint positions
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        Eigen::VectorXd& newJointPositions);
      
      /**
       * Get joint positions from inverse kinematics
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [out] newJointPositions: new joint positions
       * 
       * @return Return status of task
       */
      ReturnStatus calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<pinocchio::SE3>& endEffectorTransforms,
        Eigen::VectorXd& newJointPositions);
      
      /**
       * Get joint velocities by classic inverse Jacobian method:
       * dq/dt = J^-1 * v
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorVelocities: Velocities of 3D end effectors
       * @param [in] endEffectorTwists: Velocities of 6D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       * 
       * @warning Results might be catastrophic when robot is close to or in singularity!
       */
      ReturnStatus calculateJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorVelocities,
        const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
        Eigen::VectorXd& jointVelocities);

      /**
       * Get joint velocities by classic inverse Jacobian method:
       * dq/dt = J^-1 * v
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorVelocities: Velocities of 3D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       * 
       * @warning Results might be catastrophic when robot is close to or in singularity!
       */
      ReturnStatus calculateJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorVelocities,
        Eigen::VectorXd& jointVelocities);

      /**
       * Get joint velocities by classic inverse Jacobian method:
       * dq/dt = J^-1 * v
       * Version for models with only 6D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorTwists: Velocities of 6D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       * 
       * @warning Results might be catastrophic when robot is close to or in singularity!
       */
      ReturnStatus calculateJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
        Eigen::VectorXd& jointVelocities);

      /**
       * Get joint velocities by classic inverse damped Jacobian method:
       * dq/dt = J^T * (J * J^T + λ * I)^-1 * v
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorVelocities: Velocities of 3D end effectors
       * @param [in] endEffectorTwists: Velocities of 6D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       */
      ReturnStatus calculateDampedJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorVelocities,
        const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
        Eigen::VectorXd& jointVelocities);

      /**
       * Get joint velocities by classic inverse damped Jacobian method:
       * dq/dt = J^T * (J * J^T + λ * I)^-1 * v
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorVelocities: Velocities of 3D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       */
      ReturnStatus calculateDampedJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector3d>& endEffectorVelocities,
        Eigen::VectorXd& jointVelocities);

      /**
       * Get joint velocities by classic inverse damped Jacobian method:
       * dq/dt = J^T * (J * J^T + λ * I)^-1 * v
       * Version for models with only 6D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorTwists: Velocities of 6D end effectors
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       */
      ReturnStatus calculateDampedJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
        Eigen::VectorXd& jointVelocities);

      
      /**
       * Get poses from forward kinematics
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [out] endEffectorPositions: Positions of 3D end effectors
       * @param [out] endEffectorTransforms: Transforms of 6D end effectors
       * 
       * @return Return status of task
       * 
       * @warning Output vectors needs to be resized before calling!
       */
      ReturnStatus calculateEndEffectorPoses(const Eigen::VectorXd& actualJointPositions, 
        std::vector<Eigen::Vector3d>& endEffectorPositions,
        std::vector<pinocchio::SE3>& endEffectorTransforms);
      
      /**
       * Get poses from forward kinematics
       * Version for models with only 3D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [out] endEffectorPositions: Positions of 3D end effectors
       * 
       * @return Return status of task
       * 
       * @warning Output vectors needs to be resized before calling!
       */
      ReturnStatus calculateEndEffectorPoses(const Eigen::VectorXd& actualJointPositions, 
        std::vector<Eigen::Vector3d>& endEffectorPositions);

      /**
       * Get poses from forward kinematics
       * Version for models with only 6D end effectors
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [out] endEffectorTransforms: Transforms of 6D end effectors
       * 
       * @return Return status of task
       * 
       * @warning Output vectors needs to be resized before calling!
       */
      ReturnStatus calculateEndEffectorPoses(const Eigen::VectorXd& actualJointPositions, 
        std::vector<pinocchio::SE3>& endEffectorTransforms);

      /**
       * Get end effector velocities from forward kinematics
       * 
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] actualJointVelocities: Current joint velocities
       * @param [out] endEffectorVelocities: Velocities of 3D end effectors
       * @param [out] endEffectorTwists: Velocities of 6D end effectors
       * 
       * @return Return status of task
       * 
       * @warning Output vectors needs to be resized before calling!
       * @warning All velocities are classically defined (LOCAL_WORLD_ALIGNED)
       */
      ReturnStatus calculateEndEffectorVelocities(const Eigen::VectorXd& actualJointPositions, 
        const Eigen::VectorXd& actualJointVelocities,
        std::vector<Eigen::Vector3d>& endEffectorVelocities, 
        std::vector<Eigen::Vector<double, 6>>& endEffectorTwists);

      /**
       * Get end effector velocities from forward kinematics
       * Version for models with only 3D end effectors
       * 
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] actualJointVelocities: Current joint velocities
       * @param [out] endEffectorVelocities: Velocities of 3D end effectors
       * 
       * @return Return status of task
       * @warning Output vectors needs to be resized before calling!
       * @warning All velocities are classically defined (LOCAL_WORLD_ALIGNED)
       */
      ReturnStatus calculateEndEffectorVelocities(const Eigen::VectorXd& actualJointPositions, 
        const Eigen::VectorXd& actualJointVelocities,
        std::vector<Eigen::Vector3d>& endEffectorVelocities);

      /**
       * Get end effector velocities from forward kinematics
       * Version for models with only 6D end effectors
       * 
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] actualJointVelocities: Current joint velocities
       * @param [out] endEffectorTwists: Velocities of 6D end effectors
       * 
       * @return Return status of task
       * @warning Output vectors needs to be resized before calling!
       * @warning All velocities are classically defined (LOCAL_WORLD_ALIGNED)
       */
      ReturnStatus calculateEndEffectorVelocities(const Eigen::VectorXd& actualJointPositions, 
        const Eigen::VectorXd& actualJointVelocities, 
        std::vector<Eigen::Vector<double, 6>>& endEffectorTwists);

      /**
       * Get stacked jacobian matrix for every end effector
       * 
       * @param [in] actualJointPositions: Current joint positions
       * 
       * @return Stacked jacobian matrix
       */
      Eigen::MatrixXd getJacobian(const Eigen::VectorXd& actualJointPositions);
      
      /**
       * Check if current joint positions are within the model boundaries
       * 
       * @param [in] jointPositions: Joint positions
       * 
       * @return They are in boundaries or not
       */
      bool checkPositionBounds(const Eigen::VectorXd& jointPositions);
      
      /**
       * Check if current joint velocities are within the model boundaries
       * 
       * @param [in] jointVelocities: Joint velocities
       * 
       * @return They are in boundaries or not
       */
      bool checkVelocityBounds(const Eigen::VectorXd& jointVelocities);

      const KinematicsModelSettings& getModelSettings();

      const KinematicsInternalModelSettings& getModelInternalSettings();

      const InverseSolverSettings& getSolverSettings();

      InverseSolverType getSolverType();

      const std::string& getSolverName();

      TaskType getTaskType();

    protected:

      std::unique_ptr<ocs2::PinocchioInterface> pinocchioInterface_;
    
    private:

      Eigen::VectorXd getErrorPoses(
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        const std::vector<pinocchio::SE3>& endEffectorTransforms);

      std::unique_ptr<InverseSolverInterface> makeSolver(const std::string& solverName);


      KinematicsModelSettings modelSettings_;
      KinematicsInternalModelSettings modelInternalSettings_;
      InverseSolverSettings solverSettings_;
      std::unique_ptr<InverseSolverInterface> solverImplementation_;
  };

}; // multi_end_effector_kinematics


#endif