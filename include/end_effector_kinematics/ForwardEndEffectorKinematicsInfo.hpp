#ifndef __FORWARD_END_EFFECTOR_KINEMATICS_INFO__
#define __FORWARD_END_EFFECTOR_KINEMATICS_INFO__


#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>


namespace legged_locomotion_mpc
{
  struct ForwardEndEffectorKinematicsInfo
  {
    ForwardEndEffectorKinematicsInfo(const ocs2::PinocchioInterface& interface,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames,
      const size_t stateDim,
      const size_t inputDim);
    
    size_t numEndEffectors;
    size_t numThreeDofEndEffectors;               // 3DOF end effectors, position only
    size_t numSixDofEndEffectors;                 // 6DOF end effectors, position and orientation
    std::vector<size_t> endEffectorFrameIndices;  // indices of end-effector frames [3DOF end effectors, 6DOF end effectors]
    std::vector<size_t> endEffectorJointIndices;  // indices of end-effector parent joints [3DOF end effectors, 6DOF end effectors]
    size_t stateDim;                              // number of states needed to define the system flow map
    size_t inputDim;                              // number of inputs needed to define the system flow map
  };
  
}; // namespace legged_locomotion_mpc

#endif