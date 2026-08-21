#include <multi_end_effector_kinematics/solvers/InverseSolverInterface.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace multi_end_effector_kinematics
{
  InverseSolverInterface::InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalSettings, const InverseSolverSettings& solverSettings):
    pinocchioInterface_(&pinocchioInterface), modelInternalSettings_(modelInternalSettings), solverSettings_(solverSettings)
  {    
    const size_t outputDim = 3 * modelInternalSettings.numThreeDofEndEffectors + 6 * modelInternalSettings.numSixDofEndEffectors;
    size_t jointDofDim = 0;

    for(const auto& group : modelInternalSettings.kinematicGroups)
    {
      if(group.taskIndices.size() > group.jointVelocityIndices.size())
      {
        throw std::logic_error("InverseSolverInterface: Kinematic group is overconstrained, not supported!");
      }

      jointDofDim += group.jointVelocityIndices.size();
    }

    bool redundant = jointDofDim > outputDim;
    bool normal = jointDofDim == outputDim;
    bool damped = solverSettings.dampingCoefficient > 0.0;

    if(normal)
    {
      if(damped)
      {
        taskType_ = TaskType::NORMAL_DAMPED;
      }
      else
      {
        taskType_ = TaskType::NORMAL;
      }
    }
    else if(redundant)
    {
      if(damped)
      {
        taskType_ = TaskType::REDUNDANT_DAMPED;
      }
      else
      {
        taskType_ = TaskType::REDUNDANT;
      }
    }
    else
    {
      throw std::logic_error("InverseSolverInterface: Task is overconstrained, "
        "not supported!");
    }
  }

  TaskType InverseSolverInterface::getTaskType()
  {
    return taskType_;
  }

  Eigen::MatrixXd InverseSolverInterface::getJacobian(
    const Eigen::VectorXd& actualJointPositions)
  {
    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    pinocchio::computeJointJacobians(model, data, actualJointPositions);

    const size_t rowSize = 3 * modelInternalSettings_.numThreeDofEndEffectors + 6 * modelInternalSettings_.numSixDofEndEffectors;

    Eigen::MatrixXd jacobian(rowSize, model.nv);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const size_t rowStartIndex = 3 * i;
      jacobian.middleRows<3>(rowStartIndex) = pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED).topRows<3>();
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const size_t rowStartIndex = 6 * i - 3 * modelInternalSettings_.numThreeDofEndEffectors;
      jacobian.middleRows<6>(rowStartIndex) = pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL_WORLD_ALIGNED);
    }
    
    return jacobian;
  }
} // namespace multi_end_effector_kinematics