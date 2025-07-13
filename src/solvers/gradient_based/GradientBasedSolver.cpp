#include <lrt_inverse_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

namespace lrt_inverse_kinematics
{
  GradientBasedSolver::GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface,
    IKModelInfo& modelInfo, const IKSolverInfo& solverInfo): 
      InverseSolverInterface(pinocchioInterface, modelInfo, solverInfo)
  {
    solverType_ = SolverType::GRADIENT_BASED;
  }

  bool GradientBasedSolver::getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
    const Eigen::VectorXd& error,
    Eigen::VectorXd& jointDeltas)
  {
    Eigen::MatrixXd gradient = getGradient(actualJointPositions);
    switch(getTaskType())
    {
      case TaskType::NORMAL:
        {
          jointDeltas.noalias() = -gradient.ldlt().solve(error);
        }
        break;
      case TaskType::REDUNDANT:
        {
          Eigen::MatrixXd ggT;
          ggT.noalias() = gradient * gradient.transpose();
          jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
        }
        break;
      case TaskType::DAMPED:
        {
          Eigen::MatrixXd ggT;
          ggT.noalias() = gradient * gradient.transpose();
          ggT.diagonal().array() += solverInfo_->dampingCoefficient_;
          jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
        }
        break;
    }
    return true;
  }

  SolverType GradientBasedSolver::getSolverType()
  {
    return solverType_;
  }

};