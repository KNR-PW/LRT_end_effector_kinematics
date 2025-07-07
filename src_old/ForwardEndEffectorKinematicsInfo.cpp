#include <legged_locomotion_mpc/kinematics/ForwardEndEffectorKinematicsInfo.hpp>


namespace legged_locomotion_mpc
{

  ForwardEndEffectorKinematicsInfo::ForwardEndEffectorKinematicsInfo(
    const ocs2::PinocchioInterface& interface,
    const std::vector<std::string>& threeDofEndEffectorNames,
    const std::vector<std::string>& sixDofEndEffectorNames,
    const size_t stateDim,
    const size_t inputDim):
      numEndEffectors(threeDofEndEffectorNames.size() + sixDofEndEffectorNames.size()),
      numThreeDofEndEffectors(threeDofEndEffectorNames.size()),
      numSixDofEndEffectors(sixDofEndEffectorNames.size()),
      stateDim(stateDim),
      inputDim(inputDim)
  {

    const pinocchio::Model& model = interface.getModel();

    for (const auto& name : threeDofEndEffectorNames) 
    {
      const size_t threeDofEndEffectorFrameIndex = model.getFrameId(name);
      if(threeDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "ForwardEndEffectorKinematicsInfo error: Could not find EndEffector frame!" << std::endl;
        throw std::invalid_argument("Could not find EndEffector frame with name: " + name);
      }
      const size_t threeDofEndEffectorJointIndex = model.frames[threeDofEndEffectorFrameIndex].parentJoint;

      endEffectorFrameIndices.push_back(threeDofEndEffectorFrameIndex);
      endEffectorJointIndices.push_back(threeDofEndEffectorJointIndex);
    }

    for (const auto& name : sixDofEndEffectorNames) 
    {
      const size_t sixDofEndEffectorFrameIndex = model.getFrameId(name);
      if(sixDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "ForwardEndEffectorKinematicsInfo error: Could not find EndEffector frame!" << std::endl;
        throw std::invalid_argument("Could not find EndEffector frame with name: " + name);
      }
      const size_t sixDofEndEffectorJointIndex = model.frames[sixDofEndEffectorFrameIndex].parentJoint;

      endEffectorFrameIndices.push_back(sixDofEndEffectorFrameIndex);
      endEffectorJointIndices.push_back(sixDofEndEffectorJointIndex);
    }
  }
}