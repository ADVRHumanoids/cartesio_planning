#ifndef _GOAL_GENERATION_H_
#define _GOAL_GENERATION_H_

#include <XBotInterface/ModelInterface.h>
#include <ros/node_handle.h>
#include <validity_checker/validity_checker_context.h>
#include <goal/goal_sampler.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesio_planning/CartesioGoal.h>

class GoalGenerator
{
public:
    typedef std::shared_ptr<GoalGenerator> Ptr;

    GoalGenerator(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                  XBot::Cartesian::Planning::ValidityCheckContext& vc_context,
                  ros::NodeHandle& nh);

    void update();

private:
    ros::NodeHandle& _nh;

    XBot::Cartesian::Planning::ValidityCheckContext& _vc_context;

    XBot::Cartesian::Planning::GoalSamplerBase::Ptr _goal_sampler;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::RosServerClass::Ptr _ros_server;
    XBot::Cartesian::Planning::PositionCartesianSolver::Ptr _ik;

    ros::ServiceServer _service_a;


    bool goal_sampler_service(cartesio_planning::CartesioGoal::Request& req, cartesio_planning::CartesioGoal::Response& res);

};

#endif
