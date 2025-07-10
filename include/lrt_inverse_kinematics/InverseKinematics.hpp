#ifndef __LRT_INVERSE_KINEMATICS_HPP__
#define __LRT_INVERSE_KINEMATICS_HPP__

#include <utility>
#include <memory>

#include <Eigen/Dense>

#include <lrt_inverse_kinematics/InverseKinematicsInfo>
#include <lrt_inverse_kinematics/solvers/InvsereSolverInterface>



namespace lrt_inverse_kinematics
{

  enum class Return: uint8_t
  {
    OK = 0,
    SOLVER_ERROR = 1,
    POSITION_OUT_OF_BOUNDS = 2,
    VELOCITY_OUT_OF_BOUNDS = 3,
  };

  class InverseKinematicsSolver
  {

    public:

      InverseKinematicsSolver(IKModelInfo modelInfo, IKSolverInfo solverInfo);

      std::pair<bool, Return> calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                                               const std::vector<Eigen::Vector3d>& endEffectorPositions,
                                               const std::vector<pinocchio::SE3>& endEffectorTransforms,
                                               Eigen::VectorXd& jointDeltas);

      std::pair<bool, Return> calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
                                               const std::vector<Eigen::Vector3d>& endEffectorPositions,
                                               const std::vector<pinocchio::SE3>& endEffectorTransforms,
                                               Eigen::VectorXd& newJointPositions);

      const IKModelInfo& getModelInfo();

      const IKSolverInfo& getSolverInfo();

      SolverType getSolverType();

    private:

      IKModelInfo modelInfo_;
      IKSolverInfo solverInfo_;
      std::unique_ptr<InvsereSolverInterface> solverImplementation_;
  }

}; // lrt_inverse_kinematics


#endif