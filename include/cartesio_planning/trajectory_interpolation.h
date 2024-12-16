#ifndef TRAJECTORY_INTERPOLATION_H
#define TRAJECTORY_INTERPOLATION_H

#include <Eigen/Dense>
#include "state_space.h"

namespace XBot::Cartesian::Planning
{

struct JointTrajectory {

    struct Point {
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        double time_from_start = 0;
    };

    std::vector<Point> points;
};

JointTrajectory simpleInterpolation(StateSpace& ss,
                                    const Eigen::MatrixXd &wp,
                                    const Eigen::VectorXd &max_vel,
                                    const Eigen::VectorXd &max_acc,
                                    double dt = -1.0);

}

#endif // TRAJECTORY_INTERPOLATION_H
