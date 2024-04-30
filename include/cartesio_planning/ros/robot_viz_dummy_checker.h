#ifndef ROBOT_VIZ_DUMMY_CHECKER_H
#define ROBOT_VIZ_DUMMY_CHECKER_H

#include "robot_viz.h"
#include "../state_validity_checker.h"

namespace XBot::Cartesian::Planning
{

class RobotVizDummyChecker : public StateValidityChecker
{

public:

    RobotVizDummyChecker(RobotViz::Ptr rviz,
                         StateSpace::ConstPtr space,
                         std::string name = "viz",
                         int substate_idx = 0);

    bool checkValid(const Eigen::VectorXd &q, std::optional<Eigen::VectorXd> &qnear) const override;

private:

    RobotViz::Ptr _rviz;

};

}

#endif // ROBOT_VIZ_DUMMY_CHECKER_H
