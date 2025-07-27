#include <lrt_inverse_kinematics/solvers/gradient_based/NewtonRaphsonSolver.hpp>

namespace lrt_inverse_kinematics
{
  NewtonRaphsonSolver::NewtonRaphsonSolver(ocs2::PinocchioInterface& pinocchioInterface,
    const IKModelInternalInfo& modelInternalInfo, const IKSolverInfo& solverInfo): 
      GradientBasedSolver(pinocchioInterface, modelInternalInfo, solverInfo)
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

    const size_t rowSize = 3 * modelInternalInfo_->numThreeDofEndEffectors_ + 6 * modelInternalInfo_->numSixDofEndEffectors_;

    Eigen::MatrixXd gradient(rowSize, model.nq);

    for(size_t i = 0; i < modelInternalInfo_->numThreeDofEndEffectors_; ++i)
    {
      const size_t frameIndex = modelInternalInfo_->endEffectorFrameIndices_[i];
      const size_t rowStartIndex = 3 * i;
      gradient.middleRows<3>(rowStartIndex) = -pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL).topRows<3>();
    }

    for(size_t i = modelInternalInfo_->numThreeDofEndEffectors_; i < modelInternalInfo_->numEndEffectors_; ++i)
    {
      const size_t frameIndex = modelInternalInfo_->endEffectorFrameIndices_[i];
      const pinocchio::SE3 errorTransformInv = endEffectorTransforms[i - modelInternalInfo_->numThreeDofEndEffectors_].actInv(data.oMf[frameIndex]);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(errorTransformInv, Jlog);
      const size_t rowStartIndex = 6 * i - 3 * modelInternalInfo_->numThreeDofEndEffectors_;
      gradient.middleRows<6>(rowStartIndex) = -Jlog * pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL);
    }

    return gradient;
  }

  const std::string& NewtonRaphsonSolver::getSolverName()
  {
    return solverName_;
  }

}