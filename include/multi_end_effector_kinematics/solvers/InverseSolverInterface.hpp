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

#ifndef __INVERSE_SOLVER_INTERFACE_MULTI_END_EFFECTOR_KINEMATICS__
#define __INVERSE_SOLVER_INTERFACE_MULTI_END_EFFECTOR_KINEMATICS__

#include <vector>
#include <stdexcept>

#include <Eigen/Dense>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <multi_end_effector_kinematics/Settings.hpp>


namespace multi_end_effector_kinematics
{
  class InverseSolverInterface
  {
    public:

      /**
       * Constructor
       * Crate interface for inverse kinematics solver
       * @warning Everything in input and output is defined in base frame of reference!
       *
       * @param [in] pinocchioInterface: OCS2 pinocchio interface (created by MultiEndEffectorKinematics)
       * @param [in] modelInternalSettings: Internal kinematic model
       * @param [in] solverSettings: Settings for inverse kinematics solver
       */
      InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
        const KinematicsInternalModelSettings& modelInternalSettings, 
        const InverseSolverSettings& solverSettings);
      
      /**
       * Get one iteration of inverse kinematics algorithm (Δq) of q[n+1] = q[n] + Δq
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] error: Difference between target and current end effector 
       *  positions and transforms in local frame of reference
       * @param [in] endEffectorPositions: Positions of 3D end effectors
       * @param [in] endEffectorTransforms: Transforms of 6D end effectors
       * @param [out] jointDeltas: joint deltas Δq
       * 
       * @return Return status of task
       */
      virtual bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
        const Eigen::VectorXd& error,
        const std::vector<Eigen::Vector3d>& endEffectorPositions,
        const std::vector<pinocchio::SE3>& endEffectorTransforms,
        Eigen::VectorXd& jointDeltas) = 0;
      
      /**
       * Get joint velocities using inverse kinematics algorithm
       *
       * @param [in] actualJointPositions: Current joint positions
       * @param [in] endEffectorVelocities: Concatenated end effector velocities 
       *  (3 DoF + 6 DoF) in local frame of reference
       * @param [out] jointVelocities: Joint velocities
       * 
       * @return Return status of task
       */
      virtual bool getJointVelocities(const Eigen::VectorXd& actualJointPositions, 
        const Eigen::VectorXd& endEffectorVelocities, 
        Eigen::VectorXd& jointVelocities) = 0;

      /**
       * Get stacked jacobian matrix for every end effector
       * 
       * @param [in] actualJointPositions: Current joint positions
       * 
       * @return Stacked jacobian matrix
       */
      Eigen::MatrixXd getJacobian(const Eigen::VectorXd& actualJointPositions);

      virtual InverseSolverType getSolverType() = 0;

      virtual const std::string& getSolverName() = 0;

      TaskType getTaskType();
      
      virtual ~InverseSolverInterface() = default;

    protected:
      
      ocs2::PinocchioInterface* pinocchioInterface_;
      const KinematicsInternalModelSettings modelInternalSettings_;
      const InverseSolverSettings solverSettings_;
      
      std::string solverName_;
      InverseSolverType solverType_;
      
    private:

      TaskType taskType_;
  };
};
#endif