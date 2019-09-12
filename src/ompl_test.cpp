#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <matlogger2/matlogger2.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


int main(int argc, char ** argv)
{

    ros::init(argc, argv, "ompl_test_node");
    ros::NodeHandle nh;
    auto opt = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(opt);
    Eigen::VectorXd qmin, qmax;
    model->getJointLimits(qmin, qmax);
    const int nq = model->getJointNum();

    // construct the state space we are planning in
    auto space = std::make_shared<ob::RealVectorStateSpace>(nq);

    // set the bounds for the R^3 space
    ob::RealVectorBounds bounds(nq);
    Eigen::VectorXd::Map(bounds.low.data(), nq) = qmin;
    Eigen::VectorXd::Map(bounds.high.data(), nq) = qmax;
    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto space_info = std::make_shared<ob::SpaceInformation>(space);

//    auto constraint = std::make_shared<Manifold>();

    // Combine the ambient space and the constraint into a constrained state space.
//    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);

    // Define the constrained space information for this constrained state space.
//    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

    // state validity check
    auto isStateValid = [nq, model](const ob::State * state)
    {
        Eigen::VectorXd q = Eigen::VectorXd::Map(state->as<ob::RealVectorStateSpace::StateType>()->values, nq);
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

    space_info->setStateValidityChecker(isStateValid);

    // create a random start state
    Eigen::VectorXd sv(nq), gv(nq);
    model->getRobotState("home", sv);
    gv = (qmin + qmax)/2.0;

    // Scoped states that we will add to simple setup.
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);

    // Copy the values from the vectors into the start and goal states.
    Eigen::VectorXd::Map(start->as<ob::RealVectorStateSpace::StateType>()->values, nq) = sv;
    Eigen::VectorXd::Map(goal->as<ob::RealVectorStateSpace::StateType>()->values, nq) = gv;

    // create a problem instance
    auto pdef = std::make_shared<ob::ProblemDefinition>(space_info);

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // objective function
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(space_info));

    // create a planner for the defined space
    auto planner = std::make_shared<og::RRTstar>(space_info);

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    space_info->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->as<og::PathGeometric>()->interpolate(100);
        path->as<og::PathGeometric>()->printAsMatrix(std::cout);


        auto logger = XBot::MatLogger2::MakeLogger("/tmp/ompl_logger");
        ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states",
                                                                   10);
        sensor_msgs::JointState msg;

        ros::Publisher pub_marker = nh.advertise<visualization_msgs::MarkerArray>("obstacles",
                                                                                  10);


        msg.name = model->getEnabledJointNames();

        for(int i = 0; i < path->as<og::PathGeometric>()->getStateCount(); i++)
        {
            auto * state_i = path->as<og::PathGeometric>()->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            logger->add("state", Eigen::VectorXd::Map(state_i->values, nq));
        }

        int i = 0;
        while(ros::ok())
        {
            auto * state_i = path->as<og::PathGeometric>()->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            logger->add("state", Eigen::VectorXd::Map(state_i->values, nq));

            msg.position.assign(state_i->values, state_i->values + nq);
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
            i = i % path->as<og::PathGeometric>()->getStateCount();

        }

    }
    else
        std::cout << "No solution found" << std::endl;
}
