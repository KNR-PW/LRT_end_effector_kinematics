#ifndef __LRT_INVERSE_KINEMATICS_HPP__
#define __LRT_INVERSE_KINEMATICS_HPP__

#include <lrt_inverse_kinematics/InverseKinematicsInfo>
#include <lrt_inverse_kinematics/solvers/InvsereSolverInterface>
#include <Eigen/Dense>

namespace lrt_inverse_kinematics
{

  class InverseKinematicsSolver
  {

    public:

      InverseKinematicsSolver(IKModelInfo modelInfo, IKSolverInfo solverInfo);

      std::vector<double> calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                                               const std::vector<Eigen::Vector3d> endEffectorPositions,
                                               const std::vector<pinocchio::SE3> endEffectorTransforms)

      std::vector<double> calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
                                               const std::vector<Eigen::Vector3d> endEffectorPositions,
                                               const std::vector<pinocchio::SE3> endEffectorTransforms);

    private:

      IKModelInfo modelInfo_;
      IKSolverInfo solverInfo_;
      InvsereSolverInterface solver_impl_;
  }

}; // lrt_inverse_kinematics


#endif