#include <multi_end_effector_kinematics/Settings.hpp>

namespace multi_end_effector_kinematics
{
  std::string returnFlagToString(TaskReturnFlag flag)
  {
    std::string returnString;
    switch(flag)
    {
      case TaskReturnFlag::FINISHED:
        returnString = "FINISHED";
        break;
      case TaskReturnFlag::IN_PROGRESS:
        returnString = "IN_PROGRESS";
        break;
      case TaskReturnFlag::SOLVER_ERROR:
        returnString = "SOLVER_ERROR";
        break;
      case TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS:
        returnString = "CURRENT_POSITION_OUT_OF_BOUNDS";
        break;
      case TaskReturnFlag::CURRENT_VELOCITY_OUT_OF_BOUNDS:
        returnString = "CURRENT_VELOCITY_OUT_OF_BOUNDS";
        break;
      case TaskReturnFlag::NEW_POSITION_OUT_OF_BOUNDS:
        returnString = "NEW_POSITION_OUT_OF_BOUNDS";
        break;
      case TaskReturnFlag::NEW_VELOCITY_OUT_OF_BOUNDS:
        returnString = "NEW_VELOCITY_OUT_OF_BOUNDS";
        break;
      case TaskReturnFlag::SMALL_STEP_SIZE:
        returnString = "SMALL_STEP_SIZE";
        break;
      case TaskReturnFlag::SINGULARITY:
        returnString = "SINGULARITY";
        break;
    }
    return returnString;
  }
  std::string solverTypeToString(InverseSolverType solverType)
  {
    std::string returnString;
    switch(solverType)
    {
      case InverseSolverType::GRADIENT_BASED:
        returnString = "GRADIENT_BASED";
        break;
      case InverseSolverType::QP_BASED:
        returnString = "QP_BASED";
        break;
    }
    return returnString;
  }

  std::string taskTypeToString(TaskType taskType)
  {
    std::string returnString;
    switch(taskType)
    {
      case TaskType::NORMAL:
        returnString = "NORMAL";
        break;
      case TaskType::DAMPED:
        returnString = "DAMPED";
        break;
      case TaskType::REDUNDANT:
        returnString = "REDUNDANT";
        break;
    }
    return returnString;
  }

  std::string ReturnStatus::toString() const
  {
    std::string returnString = "Success: ";
    if(success) returnString += "True\t";
    else returnString += "No\t";
    returnString += "Flag: " + returnFlagToString(flag);
    return returnString;
  }

}

