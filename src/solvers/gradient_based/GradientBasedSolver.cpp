#include <multi_end_effector_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

namespace multi_end_effector_kinematics
{
  GradientBasedSolver::GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalInfo, const InverseSolverSettings& solverSettings): 
      InverseSolverInterface(pinocchioInterface, modelInternalInfo, solverSettings)
  {
    solverType_ = InverseSolverType::GRADIENT_BASED;
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
            ggT.diagonal().array() += solverSettings_->dampingCoefficient_;
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

  InverseSolverType GradientBasedSolver::getSolverType()
  {
    return solverType_;
  }

};