#include <multi_end_effector_kinematics/solvers/gradient_based/NewtonRaphsonSolver.hpp>

namespace multi_end_effector_kinematics
{
  NewtonRaphsonSolver::NewtonRaphsonSolver(ocs2::PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalInfo, const InverseSolverSettings& solverSettings): 
      GradientBasedSolver(pinocchioInterface, modelInternalInfo, solverSettings)
  {
    solverName_ = "NewtonRaphson";
  }

  Eigen::MatrixXd NewtonRaphsonSolver::getGradient(const Eigen::VectorXd& actualJointPositions,
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    pinocchio::computeJointJacobians(model, data, actualJointPositions);

    const size_t rowSize = 3 * modelInternalSettings_->numThreeDofEndEffectors + 6 * modelInternalSettings_->numSixDofEndEffectors;

    Eigen::MatrixXd gradient(rowSize, model.nq);

    for(size_t i = 0; i < modelInternalSettings_->numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_->endEffectorFrameIndices[i];
      const size_t rowStartIndex = 3 * i;
      gradient.middleRows<3>(rowStartIndex) = -pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL).topRows<3>();
    }

    for(size_t i = modelInternalSettings_->numThreeDofEndEffectors; i < modelInternalSettings_->numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_->endEffectorFrameIndices[i];
      const pinocchio::SE3 errorTransformInv = endEffectorTransforms[i - modelInternalSettings_->numThreeDofEndEffectors].actInv(data.oMf[frameIndex]);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(errorTransformInv, Jlog);
      const size_t rowStartIndex = 6 * i - 3 * modelInternalSettings_->numThreeDofEndEffectors;
      gradient.middleRows<6>(rowStartIndex) = -Jlog * pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL);
    }

    return gradient;
  }

  const std::string& NewtonRaphsonSolver::getSolverName()
  {
    return solverName_;
  }

}