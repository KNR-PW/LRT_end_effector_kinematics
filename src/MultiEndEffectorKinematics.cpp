#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>

#include <urdf_parser/urdf_parser.h>
#include <ocs2_pinocchio_interface/urdf.h>


namespace multi_end_effector_kinematics
{
  using namespace ocs2;
  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  MultiEndEffectorKinematics::MultiEndEffectorKinematics(const std::string urdfFilePath,
    const KinematicsModelSettings modelSettings,
    InverseSolverSettings solverSettings,
    const std::string solverName)
      :modelSettings_(modelSettings), solverSettings_(solverSettings)
  {
    using joint_pair_t = std::pair<const std::string, std::shared_ptr<::urdf::Joint>>;

    // Get model from file
    ::urdf::ModelInterfaceSharedPtr urdfTree = ::urdf::parseURDFFile(urdfFilePath);
    if (urdfTree == nullptr) {
      throw std::invalid_argument("The file " + urdfFilePath + " does not contain a valid URDF model!");
    }

    if(modelSettings_.baseLinkName != "")
    {
      // remove extraneous joints from urdf
      std::vector<std::string> jointsToRemove;
      std::vector<std::string> linksToRemove; 

      ::urdf::LinkConstSharedPtr baseLink = urdfTree ->getLink(modelSettings_.baseLinkName);

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
    
    pinocchioInterface_ = std::make_unique<PinocchioInterface>(getPinocchioInterfaceFromUrdfModel(urdfTree));

    const pinocchio::Model& model = pinocchioInterface_->getModel();

    modelInternalSettings_.numThreeDofEndEffectors = modelSettings_.threeDofEndEffectorNames.size();
    modelInternalSettings_.numSixDofEndEffectors = modelSettings_.sixDofEndEffectorNames.size();

    modelInternalSettings_.numEndEffectors = modelInternalSettings_.numThreeDofEndEffectors + modelInternalSettings_.numSixDofEndEffectors;

    for (const auto& name : modelSettings_.threeDofEndEffectorNames) {
      const size_t threeDofEndEffectorFrameIndex = model.getFrameId(name);
      if(threeDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "KinematicsModelSettings error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }

      const size_t threeDofEndEffectorJointIndex = model.frames[threeDofEndEffectorFrameIndex].parentJoint;
  
      modelInternalSettings_.endEffectorFrameIndices.push_back(threeDofEndEffectorFrameIndex);
      modelInternalSettings_.endEffectorJointIndices.push_back(threeDofEndEffectorJointIndex);
    }

    for (const auto& name : modelSettings_.sixDofEndEffectorNames) {
      const size_t sixDofEndEffectorFrameIndex = model.getFrameId(name);
      if(sixDofEndEffectorFrameIndex == model.frames.size())
      {
        std::cout << "KinematicsModelSettings error: Could not find end effector frame!" << std::endl;
        throw std::invalid_argument("Could not find end effector frame with name: " + name);
      }
      const size_t sixDofEndEffectorJointIndex = model.frames[sixDofEndEffectorFrameIndex].parentJoint;
  
      modelInternalSettings_.endEffectorFrameIndices.push_back(sixDofEndEffectorFrameIndex);
      modelInternalSettings_.endEffectorJointIndices.push_back(sixDofEndEffectorJointIndex);
    }

    solverImplementation_ = makeSolver(solverName);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointDeltas(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& jointDeltas)
  {
    assert(endEffectorPositions.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTransforms.size() == modelInternalSettings_.numSixDofEndEffectors);

    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    pinocchio::framesForwardKinematics(model, data, actualJointPositions);

    Eigen::VectorXd error = getErrorPoses(endEffectorPositions, endEffectorTransforms);
    
    if(error.norm() < solverSettings_.tolerance)
    {
      returnValue.flag = TaskReturnFlag::FINISHED;
      return returnValue; // early return
    }
        
    if(!solverImplementation_->getJointDeltas(actualJointPositions, error, endEffectorPositions,
      endEffectorTransforms, jointDeltas))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::SOLVER_ERROR;
      return returnValue; // early return
    }

    jointDeltas = solverSettings_.stepCoefficient * jointDeltas;

    if(jointDeltas.norm() < solverSettings_.minimumStepSize && error.norm() > solverSettings_.tolerance)
    {
      returnValue.success= false;
      returnValue.flag= TaskReturnFlag::SMALL_STEP_SIZE;
    }

    return returnValue;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointDeltas(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    Eigen::VectorXd& jointDeltas)
  {
    const std::vector<pinocchio::SE3> emptyTransforms;
    return calculateJointDeltas(actualJointPositions, endEffectorPositions, 
      emptyTransforms, jointDeltas);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointDeltas(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& jointDeltas)
  {
    const std::vector<Eigen::Vector3d> emptyPositions;
    return calculateJointDeltas(actualJointPositions, emptyPositions, 
      endEffectorTransforms, jointDeltas);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointPositions(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& newJointPositions)
  {

    assert(endEffectorPositions.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTransforms.size() == modelInternalSettings_.numSixDofEndEffectors);

    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    size_t iteration = 0;

    newJointPositions = actualJointPositions;
    
    while(iteration < solverSettings_.maxIterations)
    {
      pinocchio::framesForwardKinematics(model, data, newJointPositions);
      const Eigen::VectorXd error = getErrorPoses(endEffectorPositions, endEffectorTransforms);
      if(error.norm() < solverSettings_.tolerance)
      {
        returnValue.flag = TaskReturnFlag::FINISHED;
        return returnValue; // early return
      }

      Eigen::VectorXd jointDeltas;

      if(!solverImplementation_->getJointDeltas(newJointPositions, error, endEffectorPositions,
      endEffectorTransforms, jointDeltas))
      {
        returnValue.success= false;
        returnValue.flag = TaskReturnFlag::SOLVER_ERROR;
        return returnValue; // early return
      }

      jointDeltas = solverSettings_.stepCoefficient * jointDeltas;
      newJointPositions = pinocchio::integrate(model, newJointPositions, jointDeltas);

      if(jointDeltas.norm() < solverSettings_.minimumStepSize && error.norm() > solverSettings_.tolerance)
      {
        returnValue.success= false;
        returnValue.flag = TaskReturnFlag::SMALL_STEP_SIZE;
        return returnValue; // early return
      }

      if(!checkPositionBounds(newJointPositions))
      {
        returnValue.success = false;
        returnValue.flag = TaskReturnFlag::NEW_POSITION_OUT_OF_BOUNDS;
        return returnValue; // early return
      }
      
      iteration++;
    }
    return returnValue;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointPositions(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorPositions,
    Eigen::VectorXd& newJointPositions)
  {
    const std::vector<pinocchio::SE3> emptyTransforms;
    return calculateJointPositions(actualJointPositions, endEffectorPositions,
    emptyTransforms, newJointPositions);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointPositions(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<pinocchio::SE3>& endEffectorTransforms,
    Eigen::VectorXd& newJointPositions)
  {
    const std::vector<Eigen::Vector3d> emptyPositions;
    return calculateJointPositions(actualJointPositions, emptyPositions,
    endEffectorTransforms, newJointPositions);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorVelocities,
    const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
    Eigen::VectorXd& jointVelocities)
  {
    assert(endEffectorVelocities.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTwists.size() == modelInternalSettings_.numSixDofEndEffectors);

    const auto& model = pinocchioInterface_->getModel();

    assert(actualJointPositions.rows() == model.nq);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    const auto jacobian = getJacobian(actualJointPositions);

    if(jacobian.determinant() < modelSettings_.singularityThreshold)
    {
      returnValue.success= false;
      returnValue.flag = TaskReturnFlag::SINGULARITY;
      return returnValue; // early return
    }

    Eigen::VectorXd velocities;

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      velocities << endEffectorVelocities[i];
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      velocities << endEffectorTwists[i - modelInternalSettings_.numThreeDofEndEffectors];
    }

    jointVelocities.noalias() = jacobian.ldlt().solve(velocities);

    if(!checkVelocityBounds(jointVelocities))
    {
      returnValue.success = false;
      returnValue.flag= TaskReturnFlag::NEW_VELOCITY_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    returnValue.success= true;
    returnValue.flag = TaskReturnFlag::FINISHED;
    return returnValue; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorVelocities,
    Eigen::VectorXd& jointVelocities)
  {
    const std::vector<Eigen::Vector<double, 6>> emptyTwists;
    return calculateJointVelocities(actualJointPositions, endEffectorVelocities,
      emptyTwists, jointVelocities);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
    Eigen::VectorXd& jointVelocities)
  {
    const std::vector<Eigen::Vector3d> emptyVelocities;
    return calculateJointVelocities(actualJointPositions, emptyVelocities,
      endEffectorTwists, jointVelocities);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateDampedJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorVelocities,
    const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
    Eigen::VectorXd& jointVelocities)
  {

    assert(endEffectorVelocities.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTwists.size() == modelInternalSettings_.numSixDofEndEffectors);

    const auto& model = pinocchioInterface_->getModel();

    assert(actualJointPositions.rows() == model.nq);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    const Eigen::MatrixXd jacobian = getJacobian(actualJointPositions);

    Eigen::VectorXd velocities;

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      velocities << endEffectorVelocities[i];
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      velocities << endEffectorTwists[i - modelInternalSettings_.numThreeDofEndEffectors];
    }

    Eigen::MatrixXd jjT;
    jjT.noalias() = jacobian * jacobian.transpose();
    jjT.diagonal().array() += solverSettings_.dampingCoefficient;

    jointVelocities.noalias() = jacobian.transpose() * jjT.ldlt().solve(velocities);

    if(!checkVelocityBounds(jointVelocities))
    {
      returnValue.success = false;
      returnValue.flag= TaskReturnFlag::NEW_VELOCITY_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    returnValue.success= true;
    returnValue.flag = TaskReturnFlag::FINISHED;
    return returnValue; 
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateDampedJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector3d>& endEffectorVelocities,
    Eigen::VectorXd& jointVelocities)
  {
    const std::vector<Eigen::Vector<double, 6>> emptyTwists;
    return calculateDampedJointVelocities(actualJointPositions, endEffectorVelocities,
      emptyTwists, jointVelocities);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateDampedJointVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const std::vector<Eigen::Vector<double, 6>>& endEffectorTwists,
    Eigen::VectorXd& jointVelocities)
  {
    const std::vector<Eigen::Vector3d> emptyVelocities;
    return calculateDampedJointVelocities(actualJointPositions, emptyVelocities,
      endEffectorTwists, jointVelocities);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorPoses(
    const Eigen::VectorXd& actualJointPositions, 
    std::vector<Eigen::Vector3d>& endEffectorPositions,
    std::vector<pinocchio::SE3>& endEffectorTransforms)
  {

    assert(endEffectorPositions.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTransforms.size() == modelInternalSettings_.numSixDofEndEffectors);
    
    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    pinocchio::framesForwardKinematics(model, data, actualJointPositions);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const auto position = data.oMf[frameIndex].translation();
      endEffectorPositions[i] = position;
    }
    
    for(size_t i = 0; i < modelInternalSettings_.numSixDofEndEffectors; ++i)
    {
      const size_t sixDofIndex = i + modelInternalSettings_.numThreeDofEndEffectors;
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[sixDofIndex];
      const auto transform = data.oMf[frameIndex];
      endEffectorTransforms[i] = transform;
    }

    returnValue.success= true;
    returnValue.flag = TaskReturnFlag::FINISHED;
    return returnValue;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorPoses(
    const Eigen::VectorXd& actualJointPositions, 
    std::vector<Eigen::Vector3d>& endEffectorPositions)
  {
    std::vector<pinocchio::SE3> emptyTransforms;
    return calculateEndEffectorPoses(actualJointPositions, endEffectorPositions, 
      emptyTransforms);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorPoses(
    const Eigen::VectorXd& actualJointPositions, 
    std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    std::vector<Eigen::Vector3d> emptyPositions;
    return calculateEndEffectorPoses(actualJointPositions, emptyPositions, 
      endEffectorTransforms);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const Eigen::VectorXd& actualJointVelocities,
    std::vector<Eigen::Vector3d>& endEffectorVelocities, 
    std::vector<Eigen::Vector<double, 6>>& endEffectorTwists)
  {
    assert(endEffectorVelocities.size() == modelInternalSettings_.numThreeDofEndEffectors);
    assert(endEffectorTwists.size() == modelInternalSettings_.numSixDofEndEffectors);
    
    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);
    assert(actualJointVelocities.rows() == model.nv);

    ReturnStatus returnValue{true, TaskReturnFlag::IN_PROGRESS};

    if(!checkPositionBounds(actualJointPositions))
    {
      returnValue.success = false;
      returnValue.flag = TaskReturnFlag::CURRENT_POSITION_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    if(!checkVelocityBounds(actualJointVelocities))
    {
      returnValue.success = false;
      returnValue.flag= TaskReturnFlag::CURRENT_VELOCITY_OUT_OF_BOUNDS;
      return returnValue; // early return
    }

    pinocchio::forwardKinematics(model, data, actualJointPositions, actualJointVelocities);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const auto velocity = pinocchio::getFrameVelocity(model, data, frameIndex, 
        pinocchio::LOCAL_WORLD_ALIGNED).linear();
      endEffectorVelocities[i] = velocity;
    }
    
    for(size_t i = 0; i < modelInternalSettings_.numSixDofEndEffectors; ++i)
    {
      const size_t sixDofIndex = i + modelInternalSettings_.numThreeDofEndEffectors;
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[sixDofIndex];
      const auto twist = pinocchio::getFrameVelocity(model, data, frameIndex, 
        pinocchio::LOCAL_WORLD_ALIGNED).toVector();
      endEffectorTwists[i] = twist;
    }

    returnValue.success= true;
    returnValue.flag = TaskReturnFlag::FINISHED;
    return returnValue;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const Eigen::VectorXd& actualJointVelocities,
    std::vector<Eigen::Vector3d>& endEffectorVelocities)
  {
    std::vector<Eigen::Vector<double, 6>> emptyTwists;
    return calculateEndEffectorVelocities(actualJointPositions, actualJointVelocities, 
      endEffectorVelocities, emptyTwists);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  ReturnStatus MultiEndEffectorKinematics::calculateEndEffectorVelocities(
    const Eigen::VectorXd& actualJointPositions, 
    const Eigen::VectorXd& actualJointVelocities, 
    std::vector<Eigen::Vector<double, 6>>& endEffectorTwists)
  {
    std::vector<Eigen::Vector3d> emptyVelocities;
    return calculateEndEffectorVelocities(actualJointPositions, actualJointVelocities, 
      emptyVelocities, endEffectorTwists);
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::MatrixXd MultiEndEffectorKinematics::getJacobian(
    const Eigen::VectorXd& actualJointPositions)
  {
    const auto& model = pinocchioInterface_->getModel();
    auto& data = pinocchioInterface_->getData();

    assert(actualJointPositions.rows() == model.nq);

    pinocchio::computeJointJacobians(model, data, actualJointPositions);

    const size_t rowSize = 3 * modelInternalSettings_.numThreeDofEndEffectors + 6 * modelInternalSettings_.numSixDofEndEffectors;

    Eigen::MatrixXd jacobian(rowSize, model.nq);

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const size_t rowStartIndex = 3 * i;
      jacobian.middleRows<3>(rowStartIndex) = pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL).topRows<3>();
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const size_t rowStartIndex = 6 * i - 3 * modelInternalSettings_.numThreeDofEndEffectors;
      jacobian.middleRows<6>(rowStartIndex) = pinocchio::getFrameJacobian(model, data, frameIndex, pinocchio::LOCAL);
    }

    return jacobian;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  Eigen::VectorXd MultiEndEffectorKinematics::getErrorPoses(const std::vector<Eigen::Vector3d>& endEffectorPositions,
    const std::vector<pinocchio::SE3>& endEffectorTransforms)
  {
    const auto& data = pinocchioInterface_->getData();
    Eigen::VectorXd error;

    for(size_t i = 0; i < modelInternalSettings_.numThreeDofEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      error << data.oMf[frameIndex].rotation().transpose() * (endEffectorPositions[i] - data.oMf[frameIndex].translation());
    }

    for(size_t i = modelInternalSettings_.numThreeDofEndEffectors; i < modelInternalSettings_.numEndEffectors; ++i)
    {
      const size_t frameIndex = modelInternalSettings_.endEffectorFrameIndices[i];
      const pinocchio::SE3 errorTransform = data.oMf[frameIndex].actInv(endEffectorTransforms[i - modelInternalSettings_.numThreeDofEndEffectors]);
      error << pinocchio::log6(errorTransform).toVector();
    }

    return error;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool MultiEndEffectorKinematics::checkPositionBounds(const Eigen::VectorXd& newJointPositions)
  {
    const auto& model = pinocchioInterface_->getModel();
    const size_t jointNumber = model.njoints - 1; // remove root_joint

    for(size_t i = 0; i < jointNumber; ++i)
    {
      if(newJointPositions[i] > model.upperPositionLimit[i] && 
        newJointPositions[i] < model.lowerPositionLimit[i]) return false;
    }

    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  bool MultiEndEffectorKinematics::checkVelocityBounds(const Eigen::VectorXd& jointDeltas)
  {
    const auto& model = pinocchioInterface_->getModel();
    const size_t jointNumber = model.njoints - 1; // remove root_joint

    for(size_t i = 0; i < jointNumber; ++i)
    {
      if(std::abs(jointDeltas[i] > model.velocityLimit[i])) return false;
    }
    
    return true;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const pinocchio::Model& MultiEndEffectorKinematics::getPinocchioModel()
  {
    return pinocchioInterface_->getModel();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const KinematicsModelSettings& MultiEndEffectorKinematics::getModelSettings()
  {
    return modelSettings_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const KinematicsInternalModelSettings& MultiEndEffectorKinematics::getModelInternalSettings()
  {
    return modelInternalSettings_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const InverseSolverSettings& MultiEndEffectorKinematics::getSolverSettings()
  {
    return solverSettings_;
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  InverseSolverType MultiEndEffectorKinematics::getSolverType()
  {
    return solverImplementation_->getSolverType();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  const std::string& MultiEndEffectorKinematics::getSolverName()
  {
    return solverImplementation_->getSolverName();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  TaskType MultiEndEffectorKinematics::getTaskType()
  {
    return solverImplementation_->getTaskType();
  }

  /******************************************************************************************************/
  /******************************************************************************************************/
  /******************************************************************************************************/
  std::unique_ptr<InverseSolverInterface> MultiEndEffectorKinematics::makeSolver(
    const std::string& solverName)
  {
    std::unique_ptr<InverseSolverInterface> solverPtr;
    if(solverName == "NewtonRaphson")
    {
      solverPtr.reset(new NewtonRaphsonSolver(*pinocchioInterface_, modelInternalSettings_, solverSettings_));
    }
    else
    {
      throw std::invalid_argument("MultiEndEffectorKinematics: Wrong solver name!");
    }
    return solverPtr;
  }
} //  namespace multi_end_effector_kinematics