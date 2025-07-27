#ifndef __LRT_INVERSE_KINEMATICS_TEST_HPP__
#define __LRT_INVERSE_KINEMATICS_TEST_HPP__

#include <lrt_inverse_kinematics/InverseKinematics.hpp>


namespace lrt_inverse_kinematics
{

  class InverseKinematicsTest: public InverseKinematics
  {
    public:

    InverseKinematicsTest(const std::string urdfFilePath,
      const IKModelInfo modelInfo,
      IKSolverInfo solverInfo,
      const std::string solverName):
      InverseKinematics(urdfFilePath, modelInfo, solverInfo, solverName){};
    
    ocs2::PinocchioInterface* getPinocchioInterface()
    {
      return pinocchioInterface_.get();
    }
  };

}; // lrt_inverse_kinematics


#endif