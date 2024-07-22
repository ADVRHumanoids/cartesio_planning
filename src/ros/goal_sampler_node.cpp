#include <boost/algorithm/string/replace.hpp>

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

    auto wall = XBot::Collision::Shape::Box();
    wall.size << 0.1, 5, 5;
    Eigen::Affine3d w_T_wall;
    w_T_wall.setIdentity();
    w_T_wall.translation() << 0, 0, 2.5;

    coll->addCollisionShape("wall",
                            "world",
                            wall,
                            w_T_wall);

    space->addStateValidityChecker(
        std::make_shared<CollisionValidityChecker>(
            space, coll)
        );

    auto ik_pb = YAML::Load(npr.param<std::string>("goal_problem_description", ""));

    // set joint limits from params
    std::map<std::string, double> joint_limits_min, joint_limits_max;
    npr.getParam("joint_limits_min", joint_limits_min);
    npr.getParam("joint_limits_max", joint_limits_max);
    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);

    for(auto [vname1, qmin_user] : joint_limits_min)
    {
        auto vname = boost::replace_all_copy(vname1, "__", "@");
        ROS_INFO("setting qmin[%s] = %f", vname.c_str(), qmin_user);
        qmin[model->getVIndexFromVName(vname)] = qmin_user;
    }

    for(auto [vname1, qmax_user] : joint_limits_max)
    {
        auto vname = boost::replace_all_copy(vname1, "__", "@");
        ROS_INFO("setting qmax[%s] = %f", vname.c_str(), qmax_user);
        qmax[model->getVIndexFromVName(vname)] = qmax_user;
    }

    model->setJointLimits(qmin, qmax);


    // build ci
    auto params = std::make_shared<XBot::Cartesian::Parameters>(1.0);

    auto ctx = std::make_shared<XBot::Cartesian::Context>(params, model);

    XBot::Cartesian::ProblemDescription pb(ik_pb, ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    XBot::Cartesian::RosServerClass ros_api(ci);


    // build planner constraint
    YAML::Node planner_cfg;

    planner_cfg["Atlas"]["Rho"] = npr.param("atlas_rho", 2.0);
    planner_cfg["Atlas"]["Epsilon"] = npr.param("atlas_epsilon", 0.1);
    planner_cfg["Atlas"]["Alpha"] = npr.param("atlas_alpha", M_PI/16.);
    planner_cfg["Atlas"]["Exploration"] = npr.param("atlas_exploration", 0.8);

    auto constr = std::make_shared<CartesianConstraint>(space, ci, planner_cfg);

    constr->bind(space);

    std::cout << constr->jacobian(model->getJointPosition()).format(3)
              << std::endl;
    std::cout << constr->jacobian(model->getJointPosition()).jacobiSvd().singularValues() << std::endl;


    // loop
    ProfilingData::instance().reset();
    TIC();
    ROS_INFO("started looping");
    while(ros::ok())
    {
        auto q = constr->sample();

        ROS_INFO("sample");
        std::vector<std::string> failed_checks;
        while(!space->checkValid(q, &failed_checks))
        {
            ROS_INFO("sample valid");
            ros_api.run();
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

        std::string input;
        std::cin >> input;
        if(input == "reset")
        {
            ROS_INFO("reset");
            constr->reset();
        }

        // ros::Duration(0.1).sleep();
    }
    auto dt = TOC();
    ProfilingData::instance().print(std::cout, dt);




}
