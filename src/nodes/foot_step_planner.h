#ifndef FOOT_STEP_PLANNER_H
#define FOOT_STEP_PLANNER_H

#include <matlogger2/matlogger2.h>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>

#include <Eigen/Geometry>

#include <iostream>
#include <string>

#include <boost/math/constants/constants.hpp>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <cartesian_interface/markers/CartesianMarker.h>


#include <cartesio_planning/FootStepPlanner.h>

#include <ros/ros.h>

#include "state_wrapper.h"
#include "ik/position_ik_solver.h"
#include "samplers/stepSampler.h"
#include "propagators/stepPropagator.h"
#include "propagators/stepPropagatorSE2.h"
#include "validity_checker/validity_checker_context.h"
#include "utils/robot_viz.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "nodes/goal_generation.h"
#include "goal/goal_sampler.h"
#include "utils/map_converter.h"



namespace XBot { namespace Cartesian { 
    
class FootStepPlanner
{
public:
    typedef std::function<bool (const Eigen::VectorXd)> StateValidityPredicate;
    
    FootStepPlanner();  
    FootStepPlanner(const FootStepPlanner&) = delete; // Not copyable
    FootStepPlanner& operator=(const FootStepPlanner&) = delete; // Not assignable  
    
    static ompl::control::ControlSamplerPtr getSampler(const ompl::control::ControlSpace* cspace);
    
    ompl::control::StatePropagatorPtr make_propagator(const std::string propagatorType);
    
    void setStateValidityPredicate(StateValidityPredicate);
    
    void setStartAndGoalState();
    
    void run();
    
private:
    void init_load_config();
    void init_load_model();
    void init_load_position_cartesian_solver();
    void init_load_planner();
    void init_load_state_propagator();
    void init_load_validity_checker();
    void init_load_goal_generator();
    void init_load_problem_definition();
    void init_subscribe_start_goal();
    void init_planner_srv();
    void init_trajectory_publisher();
    
    void publish_tf(ros::Time T);
        
    bool planner_service(cartesio_planning::FootStepPlanner::Request& req,
                         cartesio_planning::FootStepPlanner::Response& res);
    
    bool get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req,
                                    moveit_msgs::GetPlanningScene::Response& res);
    bool apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request& req,
                                      moveit_msgs::ApplyPlanningScene::Response& res);
    
    void on_start_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    void on_goal_state_recv(const sensor_msgs::JointStateConstPtr& msg);
    
    ompl::base::PlannerPtr make_planner(std::string plannerType);
    
    int callPlanner(const double time,
                    const std::string& planner_type);
    
    bool check_state_valid(XBot::ModelInterface::ConstPtr model);
    
    void enforce_bounds(Eigen::VectorXd & q) const;
    
    ros::ServiceServer _planner_srv;
    ros::ServiceServer _get_planning_scene_srv;
    ros::ServiceServer _apply_planning_scene_srv;
    
    ros::Publisher _postural_pub;    
    ros::Publisher _trj_publisher;
    
    sensor_msgs::JointState _msg;
    
    XBot::ModelInterface::Ptr _model;
    XBot::ModelInterface::Ptr _start_model, _goal_model;
    XBot::ModelInterface::Ptr _solver_model;
    
    std::shared_ptr<Planning::PositionCartesianSolver> _solver, _goal_solver;
    
    Planning::RobotViz::Ptr _start_viz, _goal_viz;
    
    ros::Subscriber _goal_sub, _start_sub;
        
    ros::NodeHandle _nh, _nhpr;
    
    YAML::Node _planner_config;
    
    std::shared_ptr<ompl::base::CompoundStateSpace> _space;
    std::shared_ptr<ompl::control::CompoundControlSpace> _cspace;
    std::shared_ptr<ompl::control::SpaceInformation> _space_info;
    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;
    ompl::control::StatePropagatorPtr _propagator;
    ompl::base::PlannerPtr _planner;
    ompl::base::PlannerStatus _status;
    std::shared_ptr<ompl::base::Path> _path;
    
    int _ee_number;
    std::vector<std::string> _ee_name;
    
    std::shared_ptr<Planning::StateWrapper> _sw;
    
    Planning::ValidityCheckContext _vc_context;
    
    Eigen::VectorXd _qhome;
    std::vector<Eigen::VectorXd> _q_vect;
    
    std::vector<sensor_msgs::JointState> _trj;
    
    GoalGenerator::Ptr _goal_generator;
    
    std::shared_ptr<CartesianInterfaceImpl> _ci;
    
    double _z_wheel;
    
    std::shared_ptr<XBot::Converter::MapConverter> _map;
        
};

} }

#endif