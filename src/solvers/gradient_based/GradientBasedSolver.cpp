#include <lrt_inverse_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

namespace lrt_inverse_kinematics
{
  GradientBasedSolver::GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface,
    const IKModelInternalInfo& modelInternalInfo, const IKSolverInfo& solverInfo): 
      InverseSolverInterface(pinocchioInterface, modelInternalInfo, solverInfo)
  {
    solverType_ = SolverType::GRADIENT_BASED;
    switch(getTaskType())
    {
      case TaskType::NORMAL:
        {

          jointDeltasFunction_ = [&](const Eigen::MatrixXd& gradient,
                                      const Eigen::VectorXd& error, Eigen::VectorXd& jointDeltas)
          { 
            jointDeltas.noalias() = -gradient.ldlt().solve(error);
          };
        }
        break;
      case TaskType::REDUNDANT:
        {
          jointDeltasFunction_ = [&](const Eigen::MatrixXd& gradient,
                                      const Eigen::VectorXd& error, Eigen::VectorXd& jointDeltas)
          { 
            Eigen::MatrixXd ggT;
            ggT.noalias() = gradient * gradient.transpose();
            jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
          };
        }
        break;
      case TaskType::DAMPED:
        {
          jointDeltasFunction_ = [&](const Eigen::MatrixXd& gradient,
                                      const Eigen::VectorXd& error, Eigen::VectorXd& jointDeltas)
          { 
            Eigen::MatrixXd ggT;
            ggT.noalias() = gradient * gradient.transpose();
            ggT.diagonal().array() += solverInfo_->dampingCoefficient_;
            jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
          };
        }
        break;
    }
  }

  bool GradientBasedSolver::getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
    const Eigen::VectorXd& error,
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& jointDeltas)
  {
    Eigen::MatrixXd gradient = getGradient(actualJointPositions,
      endEffectorPositions, endEffectorTransforms);

    jointDeltasFunction_(gradient, error, jointDeltas);

    return true;
  }

  SolverType GradientBasedSolver::getSolverType()
  {
    return solverType_;
  }

};