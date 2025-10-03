#include <multi_end_effector_kinematics/solvers/gradient_based/NewtonRaphsonSolverAD.hpp>

namespace multi_end_effector_kinematics
{
  NewtonRaphsonSolverAD::NewtonRaphsonSolverAD(ocs2::PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalSettings, const InverseSolverSettings& solverSettings): 
      GradientBasedSolver(pinocchioInterface, modelInternalSettings, solverSettings)
  {
    solverName_ = "NewtonRaphsonAD";

    // TODO: ADD AD FUNCTION!
  }

  Eigen::MatrixXd NewtonRaphsonSolverAD::getGradient(const Eigen::VectorXd& actualJointPositions,
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    // ADD GRADIENT CALCULATION USING AD
    Eigen::MatrixXd gradient;
    return gradient;
  }

  const std::string& NewtonRaphsonSolverAD::getSolverName()
  {
    return solverName_;
  }

  ocs2::ad_vector_t NewtonRaphsonSolverAD::getErrorPositionsCppAd(
    ocs2::PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const ocs2::ad_vector_t& actualJointPositions,
    const ocs2::ad_vector_t& jointVelocityDeltas,
    const ocs2::ad_vector_t& logEndEffectorTransforms)
  {

    using SE3AD = pinocchio::SE3Tpl<ocs2::ad_scalar_t, 0>;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    ocs2::ad_vector_t jointPoitionsDelta = actualJointPositions;

    pinocchio::integrate(model, jointPoitionsDelta, jointVelocityDeltas);

    pinocchio::forwardKinematics(model, data, jointPoitionsDelta);

    ocs2::ad_vector_t error;

    for(size_t i = 0; i < modelInternalSettings_->numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_->endEffectorFrameIndices[i];
      const Eigen::Matrix<ocs2::ad_scalar_t, 3, 1> errorPosition = -data.oMf[frameIndex].rotation().transpose() * data.oMf[frameIndex].translation();
      error <<  errorPosition;
    }

    for(size_t i = modelInternalSettings_->numThreeDofEndEffectors; i < modelInternalSettings_->numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_->endEffectorFrameIndices[i];
      const size_t logTargetTransformsIndex = 6* (i - modelInternalSettings_->numThreeDofEndEffectors);
      const SE3AD targetTransform = pinocchio::exp6(logEndEffectorTransforms.middleRows<6>(logTargetTransformsIndex));
      const SE3AD errorTransform = data.oMf[frameIndex].actInv(targetTransform);
      const Eigen::Matrix<ocs2::ad_scalar_t, 6, 1> errorLogTransform = pinocchio::log6(errorTransform).toVector();
      
      error << errorLogTransform;
    }

    return error;
  }

}