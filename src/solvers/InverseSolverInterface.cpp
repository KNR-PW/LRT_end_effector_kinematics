#include <multi_end_effector_kinematics/solvers/InverseSolverInterface.hpp>


namespace multi_end_effector_kinematics
{
  InverseSolverInterface::InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
    const KinematicsInternalModelSettings& modelInternalInfo, const InverseSolverSettings& solverSettings):
    pinocchioInterface_(&pinocchioInterface), modelInternalSettings_(&modelInternalInfo), solverSettings_(&solverSettings)
  {
    const auto& model = pinocchioInterface.getModel();
    
    const size_t outputDim = 3 * modelInternalInfo.numThreeDofEndEffectors + 6 * modelInternalInfo.numSixDofEndEffectors;
    const size_t jointDofDim = model.nv;

    if(solverSettings.dampingCoefficient > 0.0) taskType_ = TaskType::DAMPED;
    else if(jointDofDim > outputDim) taskType_ = TaskType::REDUNDANT;
    else if(jointDofDim == outputDim) taskType_ = TaskType::NORMAL;
    else if(jointDofDim < outputDim)
    {
      throw std::logic_error("InverseSolverInterface: Task is overconstrained, not supported!");
    }
    
  }

  TaskType InverseSolverInterface::getTaskType()
  {
    return taskType_;
  }

}