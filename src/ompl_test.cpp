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

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;


std::shared_ptr<Planning::OmplPlanner> planner;

bool planner_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    return planner->solve(1.0);
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

    auto ik_yaml = LoadProblemDescription(LoadFrom::PARAM);
    auto ik_prob = ProblemDescription(ik_yaml, model);

    std::string impl_name = "OpenSot";
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "")
    {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }

    auto ci = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                                impl_name + "Impl",
                                                                model, ik_prob);

    if(!ci)
    {
        throw std::runtime_error("Unable to load solver '" + impl_name + "'");
    }
    else
    {
        XBot::Logger::success("Loaded solver '%s'\n", impl_name.c_str());
    }


    XBot::Cartesian::Planning::PositionCartesianSolver solver(ci, {"TCP"}); // tbd: from parameter
    RosServerClass ros_server(ci, model);

    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);
    const int nq = model->getJointNum();

    planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax);


    //    auto constraint = std::make_shared<Manifold>();

    // Combine the ambient space and the constraint into a constrained state space.
    //    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);

    // Define the constrained space information for this constrained state space.
    //    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

    // state validity check
    auto isStateValid = [model](const Eigen::VectorXd& q)
    {
        model->setJointPosition(q);
        model->update();

        Eigen::Affine3d Tee;
        model->getPose("TCP", "base_link", Tee);

        Eigen::Vector3d sphere_origin(0.5, 0.0, 0.8);
        const double radius = 0.2;

        // check validity of state defined by pos & rot
        if((Tee.translation() - sphere_origin).norm() < radius)
        {
            return false;
        }

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return true;
    };

    planner->setStateValidityChecker(isStateValid);

    // create a random start state
    Eigen::VectorXd sv(nq), gv(nq);
    model->getRobotState("home", sv);
    gv = (qmin + qmax)/2.0;

    planner->setStartAndGoalStates(sv, gv);

    planner->print();

    ros::ServiceServer service = nh.advertiseService("planner_service", planner_service);

    ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("obstacles",
                                                                              10);

    while (ros::ok())
    {

        ros_server.run();

        solver.solve();
        model->getJointPosition(gv);
        planner->setStartAndGoalStates(sv, gv);

        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "base_link";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.pose.position.x = 0.5;
        sphere.pose.position.z = 0.8;
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
            ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states",
                                                                       10);
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
                msg.position.assign(q_i.data(), q_i.data() + q_i.size());
                msg.header.stamp = ros::Time::now();
                pub.publish(msg);

                visualization_msgs::Marker sphere;
                sphere.header.frame_id = "base_link";
                sphere.header.stamp = ros::Time::now();
                sphere.type = visualization_msgs::Marker::SPHERE;
                sphere.action = visualization_msgs::Marker::ADD;
                sphere.pose.position.x = 0.5;
                sphere.pose.position.z = 0.8;
                sphere.pose.orientation.w = 1.0;
                sphere.scale.x = 0.4;
                sphere.scale.y = 0.4;
                sphere.scale.z = 0.4;
                sphere.color.a = 1.0;
                sphere.color.r = 1.0;

                visualization_msgs::MarkerArray ob_msg;
                ob_msg.markers.push_back(sphere);

                pub_marker.publish(ob_msg);

                ros::Duration(0.02).sleep();

                i++;
                i = i % planner->getSolutionPath().size();

            }

        }

        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    //    else
    //        std::cout << "No solution found" << std::endl;
}
