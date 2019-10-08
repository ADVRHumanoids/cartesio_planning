#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/ModelInterface.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include "planner/cartesio_ompl_planner.h"
#include "constraints/cartesian_constraint.h"
#include "validity_checker/validity_predicate_aggregate.h"
#include "cartesio_planning/CartesioPlanner.h"

class PlannerExecutor
{

public:

    PlannerExecutor();

    void run();

private:

    typedef std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> RsPubPtr;

    void init_load_model();
    void init_load_config();
    void init_load_planner();
    void init_load_validity_checker();
    void init_subscribe_start_goal();
    void init_trj_publisiher();
    void init_planner_srv();

    bool check_state_valid(XBot::ModelInterface::ConstPtr model);

    void on_start_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    void on_goal_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    bool planner_service(cartesio_planning::CartesioPlanner::Request& req,
                         cartesio_planning::CartesioPlanner::Response& res);

    void publish_tf(ros::Time time);

    ros::NodeHandle _nh, _nhpr;
    YAML::Node _planner_config;
    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::Planning::OmplPlanner::Ptr _planner;
    XBot::Cartesian::Planning::CartesianConstraint::Ptr _manifold;
    XBot::Cartesian::Planning::ValidityPredicateAggregate _vc_aggregate;

    ros::Subscriber _goal_sub, _start_sub;
    XBot::ModelInterface::Ptr _start_model, _goal_model;
    RsPubPtr _goal_rspub, _start_rspub;

    ros::Publisher _trj_pub;
    ros::ServiceServer _planner_srv;


};

#endif // PLANNER_EXECUTOR_H
