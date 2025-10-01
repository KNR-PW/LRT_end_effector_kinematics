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

    InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
      const KinematicsInternalModelSettings& modelInternalInfo, const InverseSolverSettings& solverSettings);

    virtual bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const Eigen::VectorXd& error,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms,
      Eigen::VectorXd& jointDeltas) = 0;

    virtual InverseSolverType getSolverType() = 0;

    virtual const std::string& getSolverName() = 0;

    TaskType getTaskType();
    
    virtual ~InverseSolverInterface() = default;

    protected:
    
    ocs2::PinocchioInterface* pinocchioInterface_;
    const KinematicsInternalModelSettings* modelInternalSettings_;
    const InverseSolverSettings* solverSettings_;
    
    std::string solverName_;
    InverseSolverType solverType_;
    
    private:

    TaskType taskType_;

  };

};
#endif