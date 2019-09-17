#include <iostream>


#include <matlogger2/matlogger2.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Empty.h>

#include "ik/position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>

#include "planner/cartesio_ompl_planner.h"
#include "goal/goal_sampler.h"
#include "constraints/cartesian_constraint.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;

XBot::Cartesian::Planning::GoalSampler::Ptr g_goal_region;
std::shared_ptr<Planning::OmplPlanner> planner;
Eigen::VectorXd sv, gv;

bool planner_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    planner->setStartAndGoalStates(sv, gv);
    return planner->solve(15.0);
}


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "ompl_test_node");
    ros::NodeHandle nh;

    auto cfg = LoadOptions(LoadFrom::PARAM);

    auto model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    auto ik_yaml_constraint = LoadProblemDescription(LoadFrom::PARAM, "problem_description_constraint");
    auto ik_yaml_goal = LoadProblemDescription(LoadFrom::PARAM, "problem_description_goal");
    auto ik_prob_constraint = ProblemDescription(ik_yaml_constraint, model);
    auto ik_prob_goal = ProblemDescription(ik_yaml_goal, model);

    std::string impl_name = "OpenSot";
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "")
    {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }

    auto ci_constraint = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                                           impl_name + "Impl",
                                                                           model, ik_prob_constraint);

    auto ci_goal = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                                     impl_name + "Impl",
                                                                     model, ik_prob_goal);


    auto solver_constraint = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci_constraint,
                                                                                                  ik_prob_constraint); // tbd: from parameter

    auto solver_goal = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci_goal,
                                                                                            ik_prob_goal); // tbd: from parameter

    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);
    qmin.head<6>() << -1, -1, -1, -2, -2, -2;
    qmax.head<6>() << -qmin.head<6>();

    auto constraint = std::make_shared<XBot::Cartesian::Planning::CartesianConstraint>(solver_constraint);
    planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, constraint);

    RosServerClass ros_server(ci_goal, model);

//    g_goal_region = std::make_shared<XBot::Cartesian::Planning::GoalSampler>(planner->getSpaceInfo(),
//                                                                                solver_goal,
//                                                                                planner->getStateWrapper());

    // state validity check
    auto isStateValid = [model](const Eigen::VectorXd& q)
    {
        model->setJointPosition(q);
        model->update();

        Eigen::Affine3d Tee;
        model->getPose("TCP_L", Tee);

        Eigen::Vector3d sphere_origin(0.4, 0.1, -0.20);
        const double radius = 0.2;

        // check validity of state defined by pos & rot
        if((Tee.translation() - sphere_origin).norm() < radius)
        {
            return false;
        }

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return true;
    };

    planner->setStateValidityPredicate(isStateValid);

    model->getRobotState("home", sv);
    gv = sv;

    ros::ServiceServer service = nh.advertiseService("planner_service", planner_service);

    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 10);

    while (ros::ok())
    {
        ros_server.run();

        solver_goal->solve();
        model->getJointPosition(gv);

        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "ci/world_odom";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = 0.4;
        sphere.pose.position.y = 0.1;
        sphere.pose.position.z = -0.2;
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = 0.4;
        sphere.scale.y = 0.4;
        sphere.scale.z = 0.4;
        sphere.color.a = 1.0;
        sphere.color.r = 1.0;

        visualization_msgs::MarkerArray ob_msg;
        ob_msg.markers.push_back(sphere);

        pub_marker.publish(ob_msg);


        if (planner->getPlannerStatus())
        {
            auto logger = XBot::MatLogger2::MakeLogger("/tmp/ompl_logger");
            ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
            sensor_msgs::JointState msg;
            msg.name = model->getEnabledJointNames();

            for(auto x : planner->getSolutionPath())
            {
                logger->add("state", x);
            }

            int i = 0;
            while(ros::ok())
            {
                auto q_i = planner->getSolutionPath().at(i);

                model->setJointPosition(q_i);
                model->update();
                ros_server.run();

                visualization_msgs::Marker sphere;
                sphere.header.frame_id = "ci/world_odom";
                sphere.header.stamp = ros::Time::now();
                sphere.type = visualization_msgs::Marker::SPHERE;
                sphere.action = visualization_msgs::Marker::ADD;
                sphere.pose.position.x = 0.4;
                sphere.pose.position.y = 0.1;
                sphere.pose.position.z = -0.20;
                sphere.pose.orientation.w = 1.0;
                sphere.scale.x = 0.4;
                sphere.scale.y = 0.4;
                sphere.scale.z = 0.4;
                sphere.color.a = 1.0;
                sphere.color.r = 1.0;

                visualization_msgs::MarkerArray ob_msg;
                ob_msg.markers.push_back(sphere);

                pub_marker.publish(ob_msg);

                ros::Duration(0.05).sleep();

                i++;
                i = i % planner->getSolutionPath().size();

            }

        }

        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

}
