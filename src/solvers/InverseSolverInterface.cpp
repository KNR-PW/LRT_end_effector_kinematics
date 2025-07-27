#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>


namespace lrt_inverse_kinematics
{
  InverseSolverInterface::InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
    const IKModelInternalInfo& modelInternalInfo, const IKSolverInfo& solverInfo):
    pinocchioInterface_(&pinocchioInterface), modelInternalInfo_(&modelInternalInfo), solverInfo_(&solverInfo)
  {
    const auto& model = pinocchioInterface.getModel();
    
    const size_t outputDim = 3 * modelInternalInfo.numThreeDofEndEffectors_ + 6 * modelInternalInfo.numSixDofEndEffectors_;
    const size_t jointDofDim = model.nv;

    if(solverInfo.dampingCoefficient_ > 0.0) taskType_ = TaskType::DAMPED;
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