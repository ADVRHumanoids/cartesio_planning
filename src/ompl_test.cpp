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

#include "constraints/cartesian_constraint.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;


std::shared_ptr<Planning::OmplPlanner> planner;
Eigen::VectorXd sv, gv;

bool planner_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    planner->setStartAndGoalStates(sv, gv);
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


    XBot::Cartesian::Planning::PositionCartesianSolver solver(ci, ik_prob); // tbd: from parameter
    RosServerClass ros_server(ci, model);

    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);
    const int nq = model->getJointNum();

    auto constraint_model = XBot::ModelInterface::getModel(cfg);
    constraint_model->setJointPosition(qhome);
    constraint_model->update();
    CartesianTask::Ptr manifold = std::make_shared<CartesianTask>("TCP", "base_link", 1, "Cartesian");
    std::vector<int> id{4};
    manifold->indices = id;
    ProblemDescription constraint_problem(manifold);
    auto constraint_ci = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                                impl_name + "Impl",
                                                                constraint_model, constraint_problem);
    XBot::Cartesian::Planning::PositionCartesianSolver::Ptr solver_constraint =
            std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(constraint_ci, constraint_problem);
    auto constraint = std::make_shared<XBot::Cartesian::Planning::CartesianConstraint>(solver_constraint);

    planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax, constraint);
    //planner = std::make_shared<Planning::OmplPlanner>(qmin, qmax);


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

    planner->setStateValidityPredicate(isStateValid);

    // create a random start state
    model->getRobotState("home", sv);
    gv = sv;
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


        if (planner->getPlannerStatus() == ompl::base::PlannerStatus::EXACT_SOLUTION ||
            planner->getPlannerStatus() == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
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

        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

}
