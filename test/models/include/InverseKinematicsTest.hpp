#include <multi_end_effector_kinematics/InverseKinematics.hpp>


namespace multi_end_effector_kinematics
{

  class InverseKinematicsTest: public InverseKinematics
  {
    InverseKinematicsTest(const std::string urdfFilePath,
      const std::string baseLinkName,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames,
      InverseSolverSettings solverSettings,
      const std::string solverName):
      InverseKinematics(urdfFilePath, baseLinkName, threeDofEndEffectorNames, 
                        sixDofEndEffectorNames, solverSettings, solverName){};
    
    ocs2::PinocchioInterface& getPinocchioInterface()
    {
      return *pinocchioInterface_;
    }
  };

}; // multi_end_effector_kinematics