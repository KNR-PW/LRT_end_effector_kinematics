#include <lrt_inverse_kinematics/InverseKinematicsInfo.hpp>

namespace lrt_inverse_kinematics
{
  IKModelInfo::IKModelInfo(const std::string urdfFilePath,
    const std::string& baseLinkName,
    const std::vector<std::string>& threeDofEndEffectorNames,
    const std::vector<std::string>& sixDofEndEffectorNames)
  {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    // Get model from file
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
    if (urdfTree == nullptr) {
      throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
    }

    if(baseLinkName == "")
    {
      // remove extraneous joints from urdf
      std::vector<std::string> jointsToRemove;
      std::vector<std::string> linksToRemove; 

      ::urdf::LinkConstSharedPtr baseLink = urdfTree ->getLink(baseLinkName);

      // Get all parents of Base Link
      while(baseLink->getParent() != nullptr)
      {
        linksToRemove.push_back(baseLink->getParent()->name);
        baseLink = baseLink->getParent();
      }

      // Get all joints before Base Link
      for (joint_pair_t& jointPair : urdfTree ->joints_) 
      {
        std::string parent_name = jointPair.second->parent_link_name;
        if (std::find(linksToRemove.begin(), linksToRemove.end(), parent_name) != linksToRemove.end()) 
        {
          jointsToRemove.push_back(jointPair.second->name);
          std::cout << jointPair.second->name << std::endl;
        }
      }

      // Remove parents and their joints from tree
      for(auto& jointToRemoveName : jointsToRemove)
      {
        urdfTree ->joints_.erase(jointToRemoveName);
      }

      for(auto& linkToRemoveName : linksToRemove)
      {
        urdfTree ->links_.erase(linkToRemoveName);
      }

      // Remove child joints and links to prepare tree for initTree()
      for(auto& link : urdfTree ->links_)
      {
        link.second->child_joints.clear();
        link.second->child_links.clear();
      }

      std::map<std::string, std::string> parent_link_tree;
      try
      {
        urdfTree ->initTree(parent_link_tree);
      }
      catch(::urdf::ParseError &e)
      {
        throw;
      }

      try
      {
        urdfTree ->initRoot(parent_link_tree);
      }
      catch(::urdf::ParseError &e)
      {
        throw;
      }
    }

    pinocchioInterface_ = ocs2::getPinocchioInterfaceFromUrdfModel(urdfTree);


    const pinocchio::Model& model = pinocchioInterface_.getModel();

    numThreeDofEndEffectors_ = threeDofEndEffectorNames.size();
    numSixDofEndEffectors_ = sixDofEndEffectorNames.size();

    numEndEffectors_ = numThreeDofEndEffectors_ + numSixDofEndEffectors_;

    for (const auto& name : threeDofEndEffectorNames) {
      const size_t threeDofEndEffectorFrameIndex = model.getFrameId(name);
      if(threeDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "IKModelInfo error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }

      const size_t threeDofEndEffectorJointIndex = model.frames[threeDofEndEffectorFrameIndex].parentJoint;
  
      endEffectorFrameIndices_.push_back(threeDofEndEffectorFrameIndex);
      endEffectorJointIndices_.push_back(threeDofEndEffectorJointIndex);
    }

    for (const auto& name : sixDofEndEffectorNames) {
      const size_t sixDofEndEffectorFrameIndex = model.getFrameId(name);
      if(sixDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "IKModelInfo error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }
      const size_t sixDofEndEffectorJointIndex = model.frames[sixDofEndEffectorFrameIndex].parentJoint;
  
      endEffectorFrameIndices_.push_back(sixDofEndEffectorFrameIndex);
      endEffectorJointIndices_.push_back(sixDofEndEffectorJointIndex);
    }

  }

};