#include <lrt_inverse_kinematics/solvers/InverseSolverInterface.hpp>


namespace lrt_inverse_kinematics
{
  InverseSolverInterface::InverseSolverInterface(ocs2::PinocchioInterface& pinocchioInterface,
    const IKModelInfo& modelInfo, const IKSolverInfo& solverInfo):
    pinocchioInterface_(&pinocchioInterface), modelInfo_(&modelInfo), solverInfo_(&solverInfo)
  {
    const auto& model = pinocchioInterface.getModel();
    
    const size_t outputDim = 3 * modelInfo.numThreeDofEndEffectors_ + 6 * modelInfo.numSixDofEndEffectors_;
    const size_t jointDofDim = model.nv;

    if(solverInfo.dampingCoefficient_ > 0.0) taskType_ = TaskType::DAMPED;
    if(jointDofDim > outputDim) taskType_ = TaskType::REDUNDANT;
    if(jointDofDim == outputDim) taskType_ = TaskType::NORMAL;
    if(jointDofDim < outputDim)
    {
      throw std::logic_error("InverseSolverInterface: Task is overconstrained, not supported!");
    }
  }

  TaskType InverseSolverInterface::getTaskType()
  {
    return taskType_;
  }

}