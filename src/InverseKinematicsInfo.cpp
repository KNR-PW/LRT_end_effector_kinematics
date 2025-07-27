#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>

namespace lrt_inverse_kinematics
{
  std::string returnFlagToString(ReturnFlag flag)
  {
    std::string returnString;
    switch(flag)
    {
      case ReturnFlag::FINISHED:
        returnString = "FINISHED";
        break;
      case ReturnFlag::IN_PROGRESS:
        returnString = "IN_PROGRESS";
        break;
      case ReturnFlag::SOLVER_ERROR:
        returnString = "SOLVER_ERROR";
        break;
      case ReturnFlag::POSITION_OUT_OF_BOUNDS:
        returnString = "POSITION_OUT_OF_BOUNDS";
        break;
      case ReturnFlag::VELOCITY_OUT_OF_BOUNDS:
        returnString = "VELOCITY_OUT_OF_BOUNDS";
        break;
      case ReturnFlag::SMALL_STEP_SIZE:
        returnString = "SMALL_STEP_SIZE";
        break;
    }
    return returnString;
  }
  std::string solverTypeToString(SolverType solverType)
  {
    std::string returnString;
    switch(solverType)
    {
      case SolverType::GRADIENT_BASED:
        returnString = "GRADIENT_BASED";
        break;
      case SolverType::QP_BASED:
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

  std::string ReturnStruct::toString()
  {
    std::string returnString = "Success: ";
    if(success_) returnString += "True\t";
    else returnString += "No\t";
    returnString += "Flag: " + returnFlagToString(flag_);
    return returnString;
  }

}

