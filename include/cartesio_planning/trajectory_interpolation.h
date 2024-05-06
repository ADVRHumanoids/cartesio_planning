#ifndef TRAJECTORY_INTERPOLATION_H
#define TRAJECTORY_INTERPOLATION_H

#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectory.h>
#include <xbot2_interface/xbotinterface2.h>

namespace XBot::Cartesian::Planning
{

trajectory_msgs::JointTrajectory simpleInterpolation(const ModelInterface& model,
                                                     const Eigen::MatrixXd &wp,
                                                     const Eigen::VectorXd &max_vel,
                                                     const Eigen::VectorXd &max_acc,
                                                     double dt = -1.0);

}

#endif // TRAJECTORY_INTERPOLATION_H
