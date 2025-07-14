#include <lrt_inverse_kinematics/InverseKinematics.hpp>


namespace lrt_inverse_kinematics
{

  InverseKinematics::InverseKinematics(const std::string urdfFilePath,
    const std::string baseLinkName,
    const std::vector<std::string>& threeDofEndEffectorNames,
    const std::vector<std::string>& sixDofEndEffectorNames,
    IKSolverInfo solverInfo,
    const std::string solverName)
      :solverInfo_(solverInfo)
  {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    // Get model from file
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
    if (urdfTree == nullptr) {
      throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
    }

    if(baseLinkName != "")
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
    
    pinocchioInterface_ = std::make_unique<ocs2::PinocchioInterface>(ocs2::getPinocchioInterfaceFromUrdfModel(urdfTree));

    const pinocchio::Model& model = pinocchioInterface_->getModel();

    modelInfo_.numThreeDofEndEffectors_ = threeDofEndEffectorNames.size();
    modelInfo_.numSixDofEndEffectors_ = sixDofEndEffectorNames.size();

    modelInfo_.numEndEffectors_ = modelInfo_.numThreeDofEndEffectors_ + modelInfo_.numSixDofEndEffectors_;

    for (const auto& name : threeDofEndEffectorNames) {
      const size_t threeDofEndEffectorFrameIndex = model.getFrameId(name);
      if(threeDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "IKModelInfo error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }

      const size_t threeDofEndEffectorJointIndex = model.frames[threeDofEndEffectorFrameIndex].parentJoint;
  
      modelInfo_.endEffectorFrameIndices_.push_back(threeDofEndEffectorFrameIndex);
      modelInfo_.endEffectorJointIndices_.push_back(threeDofEndEffectorJointIndex);
    }

    for (const auto& name : sixDofEndEffectorNames) {
      const size_t sixDofEndEffectorFrameIndex = model.getFrameId(name);
      if(sixDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "IKModelInfo error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }
      const size_t sixDofEndEffectorJointIndex = model.frames[sixDofEndEffectorFrameIndex].parentJoint;
  
      modelInfo_.endEffectorFrameIndices_.push_back(sixDofEndEffectorFrameIndex);
      modelInfo_.endEffectorJointIndices_.push_back(sixDofEndEffectorJointIndex);
    }

    solverImplementation_ = getSolver(solverName);
  }

  std::pair<bool, ReturnFlag> InverseKinematics::calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& jointDeltas)
  {
    assert(endEffectorPositions.size() == modelInfo_.numThreeDofEndEffectors_);
    assert(endEffectorTransforms.size() == modelInfo_.numSixDofEndEffectors_);

    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    std::pair<bool, ReturnFlag> returnValue = std::make_pair(true, ReturnFlag::IN_PROGRESS);

    pinocchio::framesForwardKinematics(model, data, actualJointPositions);

    Eigen::VectorXd error = getErrorPositions(actualJointPositions, endEffectorPositions, endEffectorTransforms);
    
    if(error.norm() < solverInfo_.tolerance_)
    {
      returnValue.second = ReturnFlag::FINISHED;
      return returnValue; // early return
    }
        
    if(!solverImplementation_->getJointDeltas(actualJointPositions, error, endEffectorPositions,
      endEffectorTransforms, jointDeltas))
    {
      returnValue.first = false;
      returnValue.second = ReturnFlag::SOLVER_ERROR;
      return returnValue; // early return
    }

    jointDeltas = solverInfo_.stepCoefficient_ * jointDeltas;

    if(jointDeltas.norm() < solverInfo_.minimumStepSize_ && error.norm() > solverInfo_.tolerance_)
    {
      returnValue.first = false;
      returnValue.second = ReturnFlag::SMALL_STEP_SIZE;
    }

    if(!checkVelocityBounds(jointDeltas))
    {
      returnValue.first = false;
      returnValue.second = ReturnFlag::VELOCITY_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    return returnValue;
  }

  std::pair<bool, ReturnFlag> InverseKinematics::calculateJointPositions(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& newJointPositions)
  {

    assert(endEffectorPositions.size() == modelInfo_.numThreeDofEndEffectors_);
    assert(endEffectorTransforms.size() == modelInfo_.numSixDofEndEffectors_);

    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    std::pair<bool, ReturnFlag> returnValue = std::make_pair(true, ReturnFlag::IN_PROGRESS);

    size_t iteration = 0;

    newJointPositions = actualJointPositions;

    while(iteration < solverInfo_.maxIterations_)
    {
      pinocchio::framesForwardKinematics(model, data, newJointPositions);

      const Eigen::VectorXd error = getErrorPositions(actualJointPositions, endEffectorPositions, endEffectorTransforms);

      if(error.norm() < solverInfo_.tolerance_)
      {
        returnValue.second = ReturnFlag::FINISHED;
        return returnValue; // early return
      }

      Eigen::VectorXd jointDeltas;

      if(!solverImplementation_->getJointDeltas(actualJointPositions, error, endEffectorPositions,
      endEffectorTransforms, jointDeltas))
      {
        returnValue.first = false;
        returnValue.second = ReturnFlag::SOLVER_ERROR;
        return returnValue; // early return
      }

      jointDeltas = solverInfo_.stepCoefficient_ * jointDeltas;
      pinocchio::integrate(model, newJointPositions, jointDeltas);

      if(jointDeltas.norm() < solverInfo_.minimumStepSize_ && error.norm() > solverInfo_.tolerance_)
      {
        returnValue.first = false;
        returnValue.second = ReturnFlag::SMALL_STEP_SIZE;
        return returnValue; // early return
      }

      if(!checkPositionBounds(newJointPositions))
      {
        returnValue.first = false;
        returnValue.second = ReturnFlag::POSITION_OUT_OF_BOUNDS;
        return returnValue; // early return
      }

      iteration++;
    }
    return returnValue;
  }

  Eigen::VectorXd InverseKinematics::getErrorPositions(const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    const auto& data = pinocchioInterface_->getData();
    Eigen::VectorXd error;

    for(size_t i = 0; i < modelInfo_.numThreeDofEndEffectors_; ++i)
    {
      const size_t frameIndex = modelInfo_.endEffectorFrameIndices_[i];
      error << data.oMf[frameIndex].rotation().transpose() * (endEffectorPositions[i] - data.oMf[frameIndex].translation());
    }

    for(size_t i = modelInfo_.numThreeDofEndEffectors_; i < modelInfo_.numEndEffectors_; ++i)
    {
      const size_t frameIndex = modelInfo_.endEffectorFrameIndices_[i];
      const pinocchio::SE3 errorTransform = data.oMf[frameIndex].actInv(endEffectorTransforms[i - modelInfo_.numThreeDofEndEffectors_]);
      error << pinocchio::log6(errorTransform).toVector();
    }

    return error;
  }

  bool InverseKinematics::checkPositionBounds(const Eigen::VectorXd& newJointPositions)
  {
    const auto& model = pinocchioInterface_->getModel();
    const size_t jointNumber = model.njoints - 1; // remove root_joint

    for(size_t i = 0; i < jointNumber; ++i)
    {
      const size_t jointDofStartIndex = model.idx_qs[i];
      const size_t jointDofNumber = model.nqs[i] + 1;

      for(size_t j = 0; j < jointDofNumber; ++j)
      {
        const size_t jointDofIndex = jointDofStartIndex + j;
        if(newJointPositions[jointDofIndex] > model.upperPositionLimit[jointDofIndex]) return false;
        if(newJointPositions[jointDofIndex] < model.lowerPositionLimit[jointDofIndex]) return false;
      }
    }

    return true;
  }

  bool InverseKinematics::checkVelocityBounds(const Eigen::VectorXd& jointDeltas)
  {
    const auto& model = pinocchioInterface_->getModel();
    const size_t jointNumber = model.njoints - 1; // remove root_joint

    for(size_t i = 0; i < jointNumber; ++i)
    {
      const size_t jointDofStartIndex = model.idx_vs[i];
      const size_t jointDofNumber = model.nvs[i] + 1;

      for(size_t j = 0; j < jointDofNumber; ++j)
      {
        const size_t jointDofIndex = jointDofStartIndex + j;
        if(std::abs(jointDeltas[jointDofIndex] > model.velocityLimit[jointDofIndex])) return false;
        
      }
    }
    
    return true;
  }

  const IKModelInfo& InverseKinematics::getModelInfo()
  {
    return modelInfo_;
  }

  const IKSolverInfo& InverseKinematics::getSolverInfo()
  {
    return solverInfo_;
  }

  SolverType InverseKinematics::getSolverType()
  {
    return solverImplementation_->getSolverType();
  }

  const std::string& InverseKinematics::getSolverName()
  {
    return solverImplementation_->getSolverName();
  }

  TaskType InverseKinematics::getTaskType()
  {
    return solverImplementation_->getTaskType();
  }

  std::unique_ptr<InverseSolverInterface> InverseKinematics::getSolver(
    const std::string& solverName)
  {
    std::unique_ptr<InverseSolverInterface> solverPtr;
    if(solverName == "NewtonRaphson")
    {
      solverPtr.reset(new NewtonRaphsonSolver(*pinocchioInterface_, modelInfo_, solverInfo_));
    }
    else
    {
      throw std::invalid_argument("InverseKinematics: Wrong solver name!");
    }
    return solverPtr;
  }

}