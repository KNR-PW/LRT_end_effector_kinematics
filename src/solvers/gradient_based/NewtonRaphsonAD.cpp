#include <multi_end_effector_kinematics/solvers/gradient_based/NewtonRaphsonSolverAD.hpp>

#include <pinocchio/fwd.hpp>
#include <pinocchio/codegen/cppadcg.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace multi_end_effector_kinematics
{
  using namespace ocs2;
  NewtonRaphsonSolverAD::NewtonRaphsonSolverAD(PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalSettings, const InverseSolverSettings& solverSettings): 
      GradientBasedSolver(pinocchioInterface, modelInternalSettings, solverSettings)
  {
    solverName_ = "NewtonRaphsonAD"; 

    const auto& model = pinocchioInterface.getModel();

    const size_t jointSize = model.nv;
    const size_t threeDofSize = 3 * modelInternalSettings.numThreeDofEndEffectors;
    const size_t sixDofSize = 6 * modelInternalSettings.numSixDofEndEffectors;

    // positions error function
    auto posiitonErrorApproxFunc = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
    {
      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
      const ad_vector_t actualJointPositions = x;
      const ad_vector_t endEffectorPositions = p.head(threeDofSize);
      const ad_vector_t logEndEffectorTransforms = p.tail(sixDofSize);
      y = getErrorPositionsCppAd(pinocchioInterfaceCppAd, actualJointPositions, 
        endEffectorPositions, logEndEffectorTransforms);
    };

    errorPositionsAdFunction_.reset(new CppAdInterface(posiitonErrorApproxFunc,
      jointSize, threeDofSize + sixDofSize, "newton_raphson_ad"));

    errorPositionsAdFunction_->createModels(CppAdInterface::ApproximationOrder::First, true);
  }

  Eigen::MatrixXd NewtonRaphsonSolverAD::getGradient(const Eigen::VectorXd& actualJointPositions,
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {

    const size_t rowSize = 3 * modelInternalSettings_.numThreeDofEndEffectors + 6 * modelInternalSettings_.numSixDofEndEffectors;
    vector_t positons(rowSize);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t rowStartIndex = 3 * i;
      positons.middleRows<3>(rowStartIndex) = endEffectorPositions[i];
    }
    
    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      const size_t rowStartIndex = 6 * i - 3 * modelInternalSettings_.numThreeDofEndEffectors;
      positons.middleRows<6>(rowStartIndex) = pinocchio::log6(endEffectorTransforms[i]).toVector();
    }

    Eigen::MatrixXd gradient = errorPositionsAdFunction_->getJacobian(
      actualJointPositions, positons);
    return gradient;
  }

  const std::string& NewtonRaphsonSolverAD::getSolverName()
  {
    return solverName_;
  }

  ad_vector_t NewtonRaphsonSolverAD::getErrorPositionsCppAd(
    PinocchioInterfaceCppAd& pinocchioInterfaceCppAd,
    const ad_vector_t& actualJointPositions,
    const ad_vector_t& endEffectorPositions,
    const ad_vector_t& logEndEffectorTransforms)
  {

    using SE3AD = pinocchio::SE3Tpl<ad_scalar_t, 0>;
    const auto& model = pinocchioInterfaceCppAd.getModel();
    auto& data = pinocchioInterfaceCppAd.getData();

    pinocchio::framesForwardKinematics(model, data, actualJointPositions);

    const size_t rowSize = 3 * modelInternalSettings_.numThreeDofEndEffectors + 6 * modelInternalSettings_.numSixDofEndEffectors;

    ad_vector_t error(rowSize);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const SE3AD targetTransform(data.oMf[frameIndex].rotation(), endEffectorPositions.middleRows<3>(3 * i));
      const SE3AD errorTransform = data.oMf[frameIndex].actInv(targetTransform);
      const auto errorLog6 = pinocchio::log6(errorTransform);
      const size_t rowStartIndex = 3 * i;
      error.middleRows<3>(rowStartIndex) = errorLog6.toVector().topRows<3>();
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const size_t logTargetTransformsIndex = 6 * (i - modelInternalSettings_.numThreeDofEndEffectors);
      const SE3AD targetTransform = pinocchio::exp6(logEndEffectorTransforms.middleRows<6>(logTargetTransformsIndex));
      const SE3AD errorTransform = data.oMf[frameIndex].actInv(targetTransform);
      const auto errorLog6 = pinocchio::log6(errorTransform);
      const Eigen::Matrix<ad_scalar_t, 6, 1> errorLogTransform = errorLog6.toVector();

      const size_t rowStartIndex = 6 * i - 3 * modelInternalSettings_.numThreeDofEndEffectors;
      error.middleRows<6>(rowStartIndex) = errorLogTransform;
    }

    return error;
  }

}