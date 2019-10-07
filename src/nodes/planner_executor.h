#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/ModelInterface.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include "planner/cartesio_ompl_planner.h"
#include "constraints/cartesian_constraint.h"

class PlannerExecutor
{

public:

    PlannerExecutor();

private:

    typedef std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> RsPubPtr;

    void init_load_model();
    void init_load_config();
    void init_load_planner();
    void init_load_validity_checker();
    void init_subscribe_start_goal();

    void on_start_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    void on_goal_state_recv(const sensor_msgs::JointStateConstPtr& msg);

    ros::NodeHandle _nh, _nhpr;
    YAML::Node _planner_config;
    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::Planning::OmplPlanner::Ptr _planner;
    XBot::Cartesian::Planning::CartesianConstraint::Ptr _manifold;

    ros::Subscriber _goal_sub, _start_sub;
    RsPubPtr _goal_rspub, _start_rspub;


};

#endif // PLANNER_EXECUTOR_H
