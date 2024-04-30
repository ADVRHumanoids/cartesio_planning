#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <cartesio_planning/cartesio_planning.h>
#include <cartesio_planning/ros/planning_scene_wrapper.h>
#include <cartesio_planning/ros/robot_viz.h>

#include <xbot2_interface/robotinterface2.h>

namespace XBot::Cartesian::Planning {

class PlannerExecutor
{

public:

    PlannerExecutor();

private:

    void publishMarkerStart();

    void publishMarkerGoal();

    void publishMarkerSolution();

    ros::NodeHandle _n, _npr;

    PlanningSceneWrapper::Ptr _ps;

    Eigen::VectorXd _q_start, _q_goal;

    RobotViz::Ptr _viz_start, _viz_goal, _viz_solution;

    ros::Subscriber _start_sub, _goal_sub;

    RobotInterface::Ptr _robot;

    ModelInterface::Ptr _model;

    Planner::Ptr _planner;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_main");

    XBot::Cartesian::Planning::PlannerExecutor e;

    ros::spin();
}

#endif // PLANNER_EXECUTOR_H
