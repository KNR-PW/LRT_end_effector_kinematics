#include <multi_end_effector_kinematics/MultiEndEffectorKinematics.hpp>


namespace multi_end_effector_kinematics
{

  class MultiEndEffectorKinematicsTest: public MultiEndEffectorKinematics
  {
    public:
      MultiEndEffectorKinematicsTest(const std::string urdfFilePath,
        const KinematicsModelSettings modelSettings,
        const InverseSolverSettings solverSettings,
        const std::string solverName):
        MultiEndEffectorKinematics(urdfFilePath, modelSettings, solverSettings, solverName){};
      
      ocs2::PinocchioInterface& getPinocchioInterface()
      {
        return *pinocchioInterface_;
      }
  };

}; // multi_end_effector_kinematics