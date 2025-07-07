#ifndef __LRT_INVERSE_KINEMATICS_INFO__
#define __LRT_INVERSE_KINEMATICS_INFO__


#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>


namespace lrt_inverse_kinematics
{

  struct IKModelInfo
  {
    IKModelInfo(const ocs2::PinocchioInterface& interface,
      const std::string& baseFrameName,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames);
    
    size_t baseFrameIndex;                        // base frame index (frame that forward and inverse kinematics
                                                  // are defined )
    size_t numEndEffectors;                       // Number of all end effectors
    size_t numThreeDofEndEffectors;               // 3DOF end effectors, position only
    size_t numSixDofEndEffectors;                 // 6DOF end effectors, position and orientation
    std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF end effectors, 6DOF end effectors]
    std::vector<size_t> endEffectorJointIndices;  // indices of end-effector parent joints [3DOF end effectors, 6DOF end effectors]
  };

  enum class SolverType: uint8_t
  {
    DIFFERENTIAL = 0,
    DIFFERENTIAL_DAMPED = 1,
    OPTIMIZATION_BASED = 2,
  };

  struct IKSolverInfo
  {
    unsigned max_iterations_ = 100;
    double tolerance_ = 1e-6;
    double minimum_step_size = 1e-6;
    double damping_coefficient = 1e-4; 
  };
  
}; // namespace lrt_inverse_kinematics

#endif