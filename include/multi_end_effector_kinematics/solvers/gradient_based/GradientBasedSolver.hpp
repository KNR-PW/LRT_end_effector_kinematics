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

#ifndef __GRADIENT_BASED_SOLVER_MULTI_END_EFFECTOR_KINEMATICS__
#define __GRADIENT_BASED_SOLVER_MULTI_END_EFFECTOR_KINEMATICS__

#include <functional>

#include <Eigen/Dense>

#include <multi_end_effector_kinematics/solvers/InverseSolverInterface.hpp>

namespace multi_end_effector_kinematics
{

  class GradientBasedSolver: public InverseSolverInterface
  {

    public:
    
    GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface,
      const KinematicsInternalModelSettings& modelInternalInfo, const InverseSolverSettings& solverSettings);

    bool getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
      const Eigen::VectorXd& error,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms,
      Eigen::VectorXd& jointDeltas) override final;

    virtual Eigen::MatrixXd getGradient(const Eigen::VectorXd& actualJointPositions,
      const std::vector<Eigen::Vector3d>& endEffectorPositions,
      const std::vector<pinocchio::SE3>& endEffectorTransforms) = 0;

    InverseSolverType getSolverType() override final;

    virtual ~GradientBasedSolver() = default;

    private:

    std::function<void(const Eigen::MatrixXd&,
      const Eigen::VectorXd&, Eigen::VectorXd&)> jointDeltasFunction_;

  };

};

#endif