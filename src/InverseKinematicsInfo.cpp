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
}