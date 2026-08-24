#include <multi_end_effector_kinematics/solvers/gradient_based/GradientBasedSolver.hpp>

namespace multi_end_effector_kinematics
{
  GradientBasedSolver::GradientBasedSolver(ocs2::PinocchioInterface& pinocchioInterface, const KinematicsInternalModelSettings& modelInternalSettings, const InverseSolverSettings& solverSettings):
    InverseSolverInterface(pinocchioInterface, modelInternalSettings, solverSettings)
  {
    solverType_ = InverseSolverType::GRADIENT_BASED;
  }

  bool GradientBasedSolver::solveJointDeltas(const Eigen::MatrixXd& gradient, const Eigen::VectorXd& error, Eigen::VectorXd& jointDeltas) const
  {
    if(gradient.rows() > gradient.cols())
    {
      return false;
    }

    if(solverSettings_.dampingCoefficient > 0.0)
    {
      Eigen::MatrixXd ggT;
      ggT.noalias() = gradient * gradient.transpose();
      ggT.diagonal().array() += solverSettings_.dampingCoefficient;
      jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
      return true;
    }

    if(gradient.rows() == gradient.cols())
    {
      jointDeltas.noalias() = -gradient.partialPivLu().solve(error);
      return true;
    }

    Eigen::MatrixXd ggT = gradient * gradient.transpose();
    jointDeltas.noalias() = -gradient.transpose() * ggT.ldlt().solve(error);
    return true;
  }

  bool GradientBasedSolver::solveJointVelocities(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& endEffectorVelocity, Eigen::VectorXd& jointVelocity) const
  {
    if(jacobian.rows() > jacobian.cols())
    {
      return false;
    }

    if(jacobian.rows() == jacobian.cols())
    {
      jointVelocity.noalias() = jacobian.partialPivLu().solve(endEffectorVelocity);
      return true;
    }

    Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
    jointVelocity.noalias() = jacobian.transpose() * jjT.ldlt().solve(endEffectorVelocity);
    return true;
  }

  bool GradientBasedSolver::getJointDeltas(const Eigen::VectorXd& actualJointPositions, const Eigen::VectorXd& error, const std::vector<Eigen::Vector3d>& endEffectorPositions, const std::vector<pinocchio::SE3>& endEffectorTransforms, Eigen::VectorXd& jointDeltas)
  {
    const auto& model = pinocchioInterface_->getModel();
    const Eigen::MatrixXd gradient = getGradient(actualJointPositions, endEffectorPositions, endEffectorTransforms);

    jointDeltas = Eigen::VectorXd::Zero(model.nv);

    if(modelInternalSettings_.kinematicGroups.size() == 1)
    {
      const auto& group = modelInternalSettings_.kinematicGroups.front();

      if(group.taskIndices.size() == static_cast<size_t>(error.size()) && group.jointVelocityIndices.size() == static_cast<size_t>(model.nv))
      {
        return solveJointDeltas(gradient, error, jointDeltas);
      }
    }

    for(const auto& group : modelInternalSettings_.kinematicGroups)
    {
      Eigen::MatrixXd groupGradient(group.taskIndices.size(), group.jointVelocityIndices.size());
      Eigen::VectorXd groupError(group.taskIndices.size());

      for(size_t localRowIndex = 0; localRowIndex < group.taskIndices.size(); ++localRowIndex)
      {
        const size_t globalRowIndex = group.taskIndices[localRowIndex];
        groupError[localRowIndex] = error[globalRowIndex];

        for(size_t localColumnIndex = 0; localColumnIndex < group.jointVelocityIndices.size(); ++localColumnIndex)
        {
          const size_t globalColumnIndex = group.jointVelocityIndices[localColumnIndex];
          groupGradient(localRowIndex, localColumnIndex) = gradient(globalRowIndex, globalColumnIndex);
        }
      }

      Eigen::VectorXd groupJointDeltas;
      if(!solveJointDeltas(groupGradient, groupError, groupJointDeltas))
      {
        return false;
      }

      for(size_t localColumnIndex = 0; localColumnIndex < group.jointVelocityIndices.size(); ++localColumnIndex)
      {
        jointDeltas[group.jointVelocityIndices[localColumnIndex]] = groupJointDeltas[localColumnIndex];
      }
    }

    return true;
  }

  bool GradientBasedSolver::getJointVelocities(const Eigen::VectorXd& actualJointPositions, const Eigen::VectorXd& endEffectorVelocities, Eigen::VectorXd& jointVelocities)
  {
    const auto& model = pinocchioInterface_->getModel();
    const Eigen::MatrixXd jacobian = getJacobian(actualJointPositions);

    jointVelocities = Eigen::VectorXd::Zero(model.nv);

    if(modelInternalSettings_.kinematicGroups.size() == 1)
    {
      const auto& group = modelInternalSettings_.kinematicGroups.front();

      if(group.taskIndices.size() == static_cast<size_t>(endEffectorVelocities.size()) && group.jointVelocityIndices.size() == static_cast<size_t>(model.nv))
      {
        return solveJointVelocities(jacobian, endEffectorVelocities, jointVelocities);
      }
    }

    for(const auto& group : modelInternalSettings_.kinematicGroups)
    {
      Eigen::MatrixXd groupJacobian(group.taskIndices.size(), group.jointVelocityIndices.size());
      Eigen::VectorXd groupEndEffectorVelocities(group.taskIndices.size());

      for(size_t localRowIndex = 0; localRowIndex < group.taskIndices.size(); ++localRowIndex)
      {
        const size_t globalRowIndex = group.taskIndices[localRowIndex];
        groupEndEffectorVelocities[localRowIndex] = endEffectorVelocities[globalRowIndex];

        for(size_t localColumnIndex = 0; localColumnIndex < group.jointVelocityIndices.size(); ++localColumnIndex)
        {
          const size_t globalColumnIndex = group.jointVelocityIndices[localColumnIndex];
          groupJacobian(localRowIndex, localColumnIndex) = jacobian(globalRowIndex, globalColumnIndex);
        }
      }

      Eigen::VectorXd groupJointVelocities;
      if(!solveJointVelocities(groupJacobian, groupEndEffectorVelocities, groupJointVelocities))
      {
        return false;
      }

      for(size_t localColumnIndex = 0; localColumnIndex < group.jointVelocityIndices.size(); ++localColumnIndex)
      {
        jointVelocities[group.jointVelocityIndices[localColumnIndex]] = groupJointVelocities[localColumnIndex];
      }
    }

    return true;
  }

  InverseSolverType GradientBasedSolver::getSolverType()
  {
    return solverType_;
  }

};