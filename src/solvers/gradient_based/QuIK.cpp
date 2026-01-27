#include <multi_end_effector_kinematics/solvers/gradient_based/QuIK.hpp>

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
  QuIK::QuIK(PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalSettings, const InverseSolverSettings& solverSettings): 
      GradientBasedSolver(pinocchioInterface, modelInternalSettings, solverSettings)
  {
    solverName_ = "QuIK"; 

    const auto& model = pinocchioInterface.getModel();

    const size_t jointSize = model.nv;
    const size_t threeDofSize = 3 * modelInternalSettings.numThreeDofEndEffectors;
    const size_t sixDofSize = 6 * modelInternalSettings.numSixDofEndEffectors;

    // positions error function
    auto posiitonErrorApproxFunc = [&, this](const ad_vector_t& x, const ad_vector_t& p, ad_vector_t& y) 
    {
      // initialize CppAD interface
      auto pinocchioInterfaceCppAd = pinocchioInterface.toCppAd();
  
      const ad_vector_t& actualJointPositions = x;
      const ad_vector_t& endEffectorPositions = p.head(threeDofSize);
      const ad_vector_t& logEndEffectorTransforms = p.tail(sixDofSize);
      y = getErrorPositionsCppAd(pinocchioInterfaceCppAd, actualJointPositions, 
        endEffectorPositions, logEndEffectorTransforms);
    };

    errorPositionsAdFunction_.reset(new CppAdInterface(posiitonErrorApproxFunc,
      jointSize, threeDofSize + sixDofSize, "quik_ad"));

    errorPositionsAdFunction_->createModels(CppAdInterface::ApproximationOrder::Second, false);
  
    switch(getTaskType())
    {
      case TaskType::NORMAL:
        {
          gradientFunction_ = [&](
            const Eigen::VectorXd& actualJointPositions, 
            const Eigen::VectorXd& targetPositions)
          { 
            const Eigen::VectorXd error = errorPositionsAdFunction_->getFunctionValue(
              actualJointPositions, targetPositions);

            const Eigen::MatrixXd jacobian = errorPositionsAdFunction_->getJacobian(
              actualJointPositions, targetPositions);
            
            const Eigen::VectorXd newtonRaphsonJointDelta = -jacobian.partialPivLu().solve(error);
            
            const Eigen::MatrixXd weightedHessian = errorPositionsAdFunction_->getHessian(
              newtonRaphsonJointDelta, actualJointPositions, targetPositions);

            return (jacobian + 0.5 * weightedHessian).eval();
          };
        }
        break;
      case TaskType::REDUNDANT:
        {
          gradientFunction_ = [&](
            const Eigen::VectorXd& actualJointPositions, 
            const Eigen::VectorXd& targetPositions)
          { 
            const Eigen::VectorXd error = errorPositionsAdFunction_->getFunctionValue(
              actualJointPositions, targetPositions);

            const Eigen::MatrixXd jacobian = errorPositionsAdFunction_->getJacobian(
              actualJointPositions, targetPositions);

            const Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
            
            const Eigen::VectorXd newtonRaphsonJointDelta = -jacobian.transpose() * jjT.ldlt().solve(error);
            
            const Eigen::MatrixXd weightedHessian = errorPositionsAdFunction_->getHessian(
              newtonRaphsonJointDelta, actualJointPositions, targetPositions);

            return (jacobian + 0.5 * weightedHessian).eval();
          };
        }
        break;
      case TaskType::NORMAL_DAMPED:
        {
          gradientFunction_ = [&](
            const Eigen::VectorXd& actualJointPositions, 
            const Eigen::VectorXd& targetPositions)
          { 
            const Eigen::VectorXd error = errorPositionsAdFunction_->getFunctionValue(
              actualJointPositions, targetPositions);

            const Eigen::MatrixXd jacobian = errorPositionsAdFunction_->getJacobian(
              actualJointPositions, targetPositions);

            Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
            jjT.diagonal().array() += solverSettings_.dampingCoefficient;
            
            const Eigen::VectorXd newtonRaphsonJointDelta = -jacobian.transpose() * jjT.ldlt().solve(error);
            
            const Eigen::MatrixXd weightedHessian = errorPositionsAdFunction_->getHessian(
              newtonRaphsonJointDelta, actualJointPositions, targetPositions);

            return (jacobian + 0.5 * weightedHessian).eval();
          };
        }
        break;
      case TaskType::REDUNDANT_DAMPED:
        {
          gradientFunction_ = [&](
            const Eigen::VectorXd& actualJointPositions, 
            const Eigen::VectorXd& targetPositions)
          { 
            const Eigen::VectorXd error = errorPositionsAdFunction_->getFunctionValue(
              actualJointPositions, targetPositions);

            const Eigen::MatrixXd jacobian = errorPositionsAdFunction_->getJacobian(
              actualJointPositions, targetPositions);

            Eigen::MatrixXd jjT = jacobian * jacobian.transpose();
            jjT.diagonal().array() += solverSettings_.dampingCoefficient;
            
            const Eigen::VectorXd newtonRaphsonJointDelta = -jacobian.transpose() * jjT.ldlt().solve(error);
            
            const Eigen::MatrixXd weightedHessian = errorPositionsAdFunction_->getHessian(
              newtonRaphsonJointDelta, actualJointPositions, targetPositions);

            return (jacobian + 0.5 * weightedHessian).eval();
          };
        }
        break;
    }
  }

  Eigen::MatrixXd QuIK::getGradient(const Eigen::VectorXd& actualJointPositions,
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {

    const size_t rowSize = 3 * modelInternalSettings_.numThreeDofEndEffectors + 6 * modelInternalSettings_.numSixDofEndEffectors;
    vector_t positions(rowSize);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t rowStartIndex = 3 * i;
      positions.middleRows<3>(rowStartIndex) = endEffectorPositions[i];
    }
    
    for(size_t i = 0; i < modelInternalSettings_.numSixDofEndEffectors; ++i)
    {
      const size_t rowStartIndex = 6 * i + 3 * modelInternalSettings_.numThreeDofEndEffectors;
      positions.middleRows<6>(rowStartIndex) = pinocchio::log6(endEffectorTransforms[i]).toVector();
    }

    const Eigen::MatrixXd gradient = gradientFunction_(actualJointPositions, positions);
    return gradient;
  }

  const std::string& QuIK::getSolverName()
  {
    return solverName_;
  }

  ad_vector_t QuIK::getErrorPositionsCppAd(
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
      const size_t rowStartIndex = 3 * i;
      error.middleRows<3>(rowStartIndex) = endEffectorPositions.middleRows<3>(rowStartIndex) - data.oMf[frameIndex].translation();
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