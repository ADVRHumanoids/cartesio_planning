#include <cartesio_planning/ros/robot_viz_dummy_checker.h>

using namespace XBot::Cartesian::Planning;


bool RobotVizDummyChecker::checkValid(const Eigen::VectorXd &q,
                                      std::optional<Eigen::VectorXd> &qnear) const
{
    _rviz->publishMarkers(ros::Time::now(), {});

    return true;
}
