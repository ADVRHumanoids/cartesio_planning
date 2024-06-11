#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <actionlib/server/simple_action_server.h>
#include <cartesio_planning/PlanMotionAction.h>

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

    bool publishMarkerStart();

    bool publishMarkerGoal();

    void publishMarkerSolution();

    void executePlanMotionAction(const cartesio_planning::PlanMotionGoalConstPtr& goal);

    void playTrajectoryCallback(const ros::TimerEvent& event);

    void convertToMinimalQ(trajectory_msgs::JointTrajectory& trj);

    Eigen::VectorXd jointStateToQ(const sensor_msgs::JointState& js,
                                  const Eigen::VectorXd& q0);


    ros::NodeHandle _n, _npr;

    PlanningSceneWrapper::Ptr _ps;

    Eigen::VectorXd _q_start, _q_goal;

    RobotViz::Ptr _viz_start, _viz_goal, _viz_solution;

    ros::Subscriber _start_sub, _goal_sub;

    RobotInterface::Ptr _robot;

    ModelInterface::Ptr _model, _planner_model;

    StateSpace::Ptr _ss;

    Planner::Ptr _planner;

    typedef actionlib::SimpleActionServer<cartesio_planning::PlanMotionAction> ActionServer;

    std::unique_ptr<ActionServer> _as;

    ros::Timer _playtrj_timer;

    int _playtrj_idx;

    trajectory_msgs::JointTrajectory _trj;
};

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_main");

    XBot::Cartesian::Planning::PlannerExecutor e;

    ros::spin();
}

#endif // PLANNER_EXECUTOR_H
