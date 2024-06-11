#include <cartesio_planning/ros/robot_viz.h>

#include <cartesio_planning/state_space.h>

#include <cartesio_planning/state_validity_checker/collision.h>

#include <xbot2_interface/ros/config_from_param.hpp>

#include <cartesio_planning/constraints/cartesian_constraint.h>

#include <cartesian_interface/ros/RosServerClass.h>

#include "../src/impl/profiling.hxx"  // todo make it public api

using namespace XBot::Cartesian::Planning;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_sampler_node");

    ros::NodeHandle npr("~");

    auto cfg = XBot::Utils::ConfigOptionsFromParamServer();

    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);
    model->setJointPosition(model->getRobotState("home"));
    model->update();

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    auto coll = std::make_shared<XBot::Collision::CollisionModel>(model);

    space->addStateValidityChecker(
        std::make_shared<CollisionValidityChecker>(
            space, coll)
        );

    auto ik_pb = YAML::Load(npr.param<std::string>("goal_problem_description", ""));

    auto params = std::make_shared<XBot::Cartesian::Parameters>(1.0);

    auto ctx = std::make_shared<XBot::Cartesian::Context>(params, model);

    XBot::Cartesian::ProblemDescription pb(ik_pb, ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);



    YAML::Node planner_cfg;

    planner_cfg["Atlas"]["Rho"] = npr.param("atlas_rho", 2.0);
    planner_cfg["Atlas"]["Epsilon"] = npr.param("atlas_epsilon", 0.1);
    planner_cfg["Atlas"]["Alpha"] = npr.param("atlas_alpha", M_PI/16.);
    planner_cfg["Atlas"]["Exploration"] = npr.param("atlas_exploration", 0.8);

    auto constr = std::make_shared<CartesianConstraint>(ci, planner_cfg);

    constr->bind(space);

    std::cout << constr->jacobian(model->getJointPosition()).format(3)
              << std::endl;
    std::cout << constr->jacobian(model->getJointPosition()).jacobiSvd().singularValues() << std::endl;

    XBot::Cartesian::RosServerClass ros_api(ci);

    ProfilingData::instance().reset();
    TIC();
    while(ros::ok())
    {
        auto q = constr->sample();

        while(!space->checkValid(q))
        {
            q = constr->sample();
        }

        // constr->refine(q);

        // if(!space->checkValid(q))
        // {
        //     ROS_ERROR("diocane");
        // }

        model->setJointPosition(q);

        model->update();

        ros_api.run();

        // ros::Duration(0.1).sleep();
    }
    auto dt = TOC();
    ProfilingData::instance().print(std::cout, dt);




}
