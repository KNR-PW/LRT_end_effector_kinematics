#include <lrt_inverse_kinematics/InverseKinematics.hpp>


namespace lrt_inverse_kinematics
{

  class InverseKinematicsTest: public InverseKinematics
  {
    InverseKinematicsTest(const std::string urdfFilePath,
      const std::string baseLinkName,
      const std::vector<std::string>& threeDofEndEffectorNames,
      const std::vector<std::string>& sixDofEndEffectorNames,
      IKSolverInfo solverInfo,
      const std::string solverName):
      InverseKinematics(urdfFilePath, baseLinkName, threeDofEndEffectorNames, 
                        sixDofEndEffectorNames, solverInfo, solverName){};
    
    ocs2::PinocchioInterface& getPinocchioInterface()
    {
      return *pinocchioInterface_;
    }
  };

}; // lrt_inverse_kinematics