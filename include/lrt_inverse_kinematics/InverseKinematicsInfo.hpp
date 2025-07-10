#ifndef __LRT_INVERSE_KINEMATICS_INFO__
#define __LRT_INVERSE_KINEMATICS_INFO__


#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>


namespace lrt_inverse_kinematics
{

  struct IKModelInfo
  {
    IKModelInfo(const std::string urdfPath,
      const std::string& baseFrameName,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames);


    ocs2::PinocchioInterface& pinocchioInterface_; // ocs2 pinocchio interface
    
    size_t baseFrameIndex_;                        // base frame index (frame that forward and inverse kinematics are defined)
    size_t numEndEffectors_;                       // Number of all end effectors
    size_t numThreeDofEndEffectors_;               // 3DOF end effectors, position only
    size_t numSixDofEndEffectors_;                 // 6DOF end effectors, position and orientation
    std::vector<size_t> endEffectorFrameIndices_;  // indices of end-effector frames [3DOF end effectors, 6DOF end effectors]
    std::vector<size_t> endEffectorJointIndices_;  // indices of end-effector parent joints [3DOF end effectors, 6DOF end effectors]

  };

  struct IKSolverInfo
  {
    unsigned maxIterations_ = 100;
    double tolerance_ = 1e-6;
    double minimumStepSize_ = 1e-6;
    double dampingCoefficient_ = 1e-4; 
  };
  
}; // namespace lrt_inverse_kinematics

#endif