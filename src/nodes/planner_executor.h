#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <XBotInterface/ModelInterface.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include "constraints/cartesian_constraint.h"
#include "planner/cartesio_ompl_planner.h"
#include "utils/robot_viz.h"
#include "validity_checker/validity_predicate_aggregate.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/validity_checker_context.h"

#include "cartesio_planning/CartesioPlanner.h"

class PlannerExecutor
{

public:

    PlannerExecutor();

    void run();

private:

    typedef std::shared_ptr<XBot::Cartesian::Planning::PlanningSceneWrapper> PlanningScenePtr;

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
    XBot::Cartesian::Planning::ValidityCheckContext _vc_context;

    ros::Subscriber _goal_sub, _start_sub;
    XBot::ModelInterface::Ptr _start_model, _goal_model;
    XBot::Cartesian::Planning::RobotViz::Ptr _start_viz, _goal_viz;

    ros::Publisher _trj_pub;
    ros::ServiceServer _planner_srv;


};

#endif // PLANNER_EXECUTOR_H
