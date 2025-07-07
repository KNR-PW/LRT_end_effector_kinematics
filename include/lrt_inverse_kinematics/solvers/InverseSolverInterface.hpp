#ifndef __LRT_INVERSE_SOLVER_INTERFACE__
#define __LRT_INVERSE_SOLVER_INTERFACE__

#include <vector>
#include <Eigen/Dense>
#include <pinocchio/fwd>
#include <pinocchio/algorithm/kinematics.hpp>

class InverseSolverInterface
{

  public:

  virtual Eigen::VectorXd calculateJointDeltas(const Eigen::VectorXd& actualJointPositions, 
                                               const std::vector<Eigen::Vector3d> endEffectorPositions,
                                               const std::vector<pinocchio::SE3> endEffectorTransforms) = 0;

  virtual ~InverseSolverInterface() = default;

};


#endif