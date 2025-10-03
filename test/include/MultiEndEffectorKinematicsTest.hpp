// Copyright (c) 2025, Koło Naukowe Robotyków
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Authors: Bartłomiej Krajewski (https://github.com/BartlomiejK2)
 */

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