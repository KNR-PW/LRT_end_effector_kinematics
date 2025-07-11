#include <lrt_inverse_kinematics/solvers/differential_solvers/DifferentialSolver.hpp>

namespace lrt_inverse_kinematics
{

  Eigen::VectorXd DifferentialSolver::getJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                               const Eigen::VectorXd& error)
  {
    Eigen::VectorXd jointDeltas;
    Eigen::MatrixXd gradient = getGradient(actualJointPositions);
    switch(taskType_)
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
    return jointDeltas;
  }

};