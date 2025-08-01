#ifndef __LRT_INVERSE_KINEMATICS_INFO__
#define __LRT_INVERSE_KINEMATICS_INFO__


#include <string>
#include <vector>
#include <stdexcept>
#include <memory>

#include <urdf_parser/urdf_parser.h>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/urdf.h>


namespace lrt_inverse_kinematics
{

  struct IKSolverInfo
  {
    unsigned maxIterations_ = 100;
    double tolerance_ = 1e-6;
    double minimumStepSize_ = 1e-6;
    double dampingCoefficient_ = 1e-4; 
    double stepCoefficient_ = 0.5; // > 0
  };

  struct IKModelInfo
  {
    std::string baseLinkName_;
    std::vector<std::string> threeDofEndEffectorNames_;
    std::vector<std::string> sixDofEndEffectorNames_;
  };

  struct IKModelInternalInfo
  {
    size_t baseFrameIndex_;                        // base frame index (frame that forward and inverse kinematics are defined)
    size_t numEndEffectors_;                       // Number of all end effectors
    size_t numThreeDofEndEffectors_;               // 3DOF end effectors, position only
    size_t numSixDofEndEffectors_;                 // 6DOF end effectors, position and orientation
    std::vector<size_t> endEffectorFrameIndices_;  // indices of end-effector frames [3DOF end effectors, 6DOF end effectors]
    std::vector<size_t> endEffectorJointIndices_;  // indices of end-effector parent joints [3DOF end effectors, 6DOF end effectors]
  };


  enum class ReturnFlag
  {
    FINISHED = 0, // true
    IN_PROGRESS = 1,

    SOLVER_ERROR = 2, //  false
    POSITION_OUT_OF_BOUNDS = 3,
    VELOCITY_OUT_OF_BOUNDS = 4,
    SMALL_STEP_SIZE = 5,
  };

  enum class SolverType
  {
    GRADIENT_BASED = 0,
    QP_BASED = 1,
  };

  enum class TaskType
  {
    NORMAL = 0,
    REDUNDANT = 1,
    DAMPED = 2,
  };

  struct ReturnStruct
  {
    bool success_;
    ReturnFlag flag_;

    std::string toString();
  };

  std::string returnFlagToString(ReturnFlag flag);

  std::string solverTypeToString(SolverType solverType);

  std::string taskTypeToString(TaskType taskType);
  
}; // namespace lrt_inverse_kinematics

#endif