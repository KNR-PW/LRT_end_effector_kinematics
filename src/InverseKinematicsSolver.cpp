#include <lrt_inverse_kinematics/InverseKinematicsSolver.hpp>


namespace lrt_inverse_kinematics
{

  InverseKinematicsSolver::InverseKinematicsSolver(IKModelInfo modelInfo,
    IKSolverInfo solverInfo,
    std::string solverName):
    modelInfo_(modelInfo), solverInfo_(solverInfo)
  {
    
  }

  std::pair<bool, ReturnFlag> InverseKinematicsSolver::calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& jointDeltas)
  {
    assert(endEffectorPositions.size() == modelInfo_.numThreeDofEndEffectors_);
    assert(endEffectorTransforms.size() == modelInfo_.numSixDofEndEffectors_);

    const auto& model = modelInfo_.pinocchioInterface_.getModel();
    auto& data = modelInfo_.pinocchioInterface_.getData();

    assert(actualJointPositions.rows() == model.nq);

    std::pair<bool, ReturnFlag> returnValue = std::make_pair(true, ReturnFlag::IN_PROGRESS);

    pinocchio::framesForwardKinematics(model, data, actualJointPositions);

    Eigen::VectorXd error = getErrorPositions(actualJointPositions, endEffectorPositions, endEffectorTransforms);
    
    if(error.norm() < solverInfo_.tolerance_)
    {
      returnValue.second = ReturnFlag::FINISHED;
      return returnValue; // early return
    }

    jointDeltas = solverInfo_.stepCoefficient_ * solverImplementation_->getJointDeltas(actualJointPositions, error);

    if(jointDeltas.norm() < solverInfo_.minimumStepSize_ && error.norm() > solverInfo_.tolerance_)
    {
      returnValue.first = false;
      returnValue.second = ReturnFlag::SMALL_STEP_SIZE;
    }

    return returnValue;
  }

  std::pair<bool, ReturnFlag> InverseKinematicsSolver::calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& newJointPositions)
  {

    assert(endEffectorPositions.size() == modelInfo_.numThreeDofEndEffectors_);
    assert(endEffectorTransforms.size() == modelInfo_.numSixDofEndEffectors_);

    const auto& model = modelInfo_.pinocchioInterface_.getModel();
    auto& data = modelInfo_.pinocchioInterface_.getData();

    assert(actualJointPositions.rows() == model.nq);

    std::pair<bool, ReturnFlag> returnValue = std::make_pair(true, ReturnFlag::IN_PROGRESS);

    size_t iteration = 0;
    Eigen::VectorXd error = Eigen::VectorXd::Ones(3 * modelInfo_.numThreeDofEndEffectors_ + 6 * modelInfo_.numSixDofEndEffectors_);

    newJointPositions = actualJointPositions;

    while(iteration < solverInfo_.maxIterations_)
    {
      pinocchio::framesForwardKinematics(model, data, newJointPositions);

      error = getErrorPositions(actualJointPositions, endEffectorPositions, endEffectorTransforms);

      if(error.norm() < solverInfo_.tolerance_)
      {
        returnValue.second = ReturnFlag::FINISHED;
        return returnValue; // early return
      }

      const Eigen::VectorXd jointDeltas = solverInfo_.stepCoefficient_ * solverImplementation_->getJointDeltas(newJointPositions, error);

      newJointPositions += jointDeltas;

      if(jointDeltas.norm() < solverInfo_.minimumStepSize_ && error.norm() > solverInfo_.tolerance_)
      {
        returnValue.first = false;
        returnValue.second = ReturnFlag::SMALL_STEP_SIZE;
        return returnValue; // early return
      }

    }
    return returnValue;
  }

  Eigen::VectorXd InverseKinematicsSolver::getErrorPositions(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    const auto& data = modelInfo_.pinocchioInterface_.getData();
    Eigen::VectorXd error;

    for(size_t i = 0; i < modelInfo_.numThreeDofEndEffectors_; ++i)
    {
      size_t frameIndex = modelInfo_.endEffectorFrameIndices_[i];
      const Eigen::Vector3d acutalPosition = data.oMf[modelInfo_.baseFrameIndex_].rotation() * data.oMf[frameIndex].translation();
      error << endEffectorPositions[i] - acutalPosition;
    }

    for(size_t i = modelInfo_.numThreeDofEndEffectors_; i < modelInfo_.numEndEffectors_; ++i)
    {
      size_t frameIndex = modelInfo_.endEffectorFrameIndices_[i];
      const pinocchio::SE3 actualTransform = data.oMf[modelInfo_.baseFrameIndex_].actInv(data.oMf[frameIndex]);
      const pinocchio::SE3 errorTransform = actualTransform.actInv(endEffectorTransforms[i - modelInfo_.numThreeDofEndEffectors_]);
      error << pinocchio::log6(errorTransform).toVector();
    }

    return error;
  }

  bool InverseKinematicsSolver::checkPositionBounds(const Eigen::VectorXd& newJointPositions)
  {
    const auto& model = modelInfo_.pinocchioInterface_.getModel();

    for(int i = 0)
  }

  const IKModelInfo& InverseKinematicsSolver::getModelInfo()
  {
    return modelInfo_;
  }

  const IKSolverInfo& InverseKinematicsSolver::getSolverInfo()
  {
    return solverInfo_;
  }

  SolverType InverseKinematicsSolver::getSolverType()
  {
    return solverImplementation_->getSolverType();
  }

  const std::string& InverseKinematicsSolver::getSolverName()
  {
    return solverImplementation_->getSolverName();
  }

}