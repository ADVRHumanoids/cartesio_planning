#include "ik/position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>
#include <validity_checker/stability/stability_detection.h>
#include <validity_checker/validity_predicate_aggregate.h>
#include <functional>
#include <validity_checker/collisions/planning_scene_wrapper.h>
#include "goal/goal_sampler.h"

#include "cartesio_planning/CartesioGoal.h"
#include "cartesio_planning/SetContactFrames.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;

XBot::Cartesian::Planning::GoalSamplerBase::Ptr goal_sampler;
XBot::ModelInterface::Ptr model;
XBot::Cartesian::Planning::ConvexHullStability::PolygonFrames polyframes;
XBot::Cartesian::Planning::ConvexHullStability::Ptr ch;
XBot::Cartesian::Planning::ConvexHullVisualization::Ptr ch_viz;

bool goal_sampler_service(cartesio_planning::CartesioGoal::Request& req, cartesio_planning::CartesioGoal::Response& res)
{
    Eigen::VectorXd q;
    if(!goal_sampler->sampleGoal(q, req.time)){
        res.status.val = res.status.TIMEOUT;
        res.status.msg.data = "TIMEOUT";
    }
    else
    {
        res.status.val = res.status.EXACT_SOLUTION;
        res.status.msg.data = "EXACT_SOLUTION";

        res.sampled_goal.name = model->getEnabledJointNames();
        res.sampled_goal.position.resize(q.size());
        Eigen::VectorXd::Map(&res.sampled_goal.position[0], q.size()) = q;
        res.sampled_goal.header.stamp = ros::Time::now();
    }




    return true;
}

bool set_contact_frames_service(cartesio_planning::SetContactFrames::Request& req, cartesio_planning::SetContactFrames::Response& res)
{
    polyframes.clear();
    for(unsigned int i = 0; i < req.frames_in_contact.size(); ++i)
        polyframes.push_back(req.frames_in_contact[i]);


    if(req.action.data() == req.CLEAR)
    {
        ch->setPolygonFrames(polyframes);
        res.result.data = true;
    }
    else if(req.action.data() == req.ADD)
    {
        ch->addPolygonFrames(polyframes);
        res.result.data = true;
    }
    else if(req.action.data() == req.REMOVE)
    {
        ch->removePolygonFrames(polyframes);
        res.result.data = true;
    }

    res.result.data = false;
    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "position_ik_test");
    ros::NodeHandle n("~");
    ros::NodeHandle nh("planner");

    // obtain robot model from param server
    auto cfg = LoadOptions(LoadFrom::PARAM);
    model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome, qmin, qmax;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    model->getJointLimits(qmin, qmax);

    Eigen::Affine3d T;
    model->getPose("l_sole",T);
    model->setFloatingBasePose(T.inverse());
    model->update();


    // obtain ci object from param server
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

    // construct position-level ik solver
    auto solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci, ik_prob);


    ros::ServiceServer service_a = nh.advertiseService("goal_sampler_service", goal_sampler_service);

    // construct ros server class (mainly for markers and to publish TFs)
    RosServerClass::Ptr ros_server = std::make_shared<RosServerClass>(ci, model);
    solver->setRosServerClass(ros_server);


    polyframes.push_back("l_foot_lower_left_link");
    polyframes.push_back("l_foot_lower_right_link");
    polyframes.push_back("l_foot_upper_left_link");
    polyframes.push_back("l_foot_upper_right_link");
    polyframes.push_back("r_foot_lower_left_link");
    polyframes.push_back("r_foot_lower_right_link");
    polyframes.push_back("r_foot_upper_left_link");
    polyframes.push_back("r_foot_upper_right_link");
    ch = std::make_shared<XBot::Cartesian::Planning::ConvexHullStability>(model, polyframes);
    ch_viz = std::make_shared<XBot::Cartesian::Planning::ConvexHullVisualization>(model, *ch);
    ros::ServiceServer service_b = nh.advertiseService("set_contact_frames_service", set_contact_frames_service);


    XBot::Cartesian::Planning::PlanningSceneWrapper collision(model);

    auto check_collision = [&collision]()
    {
        collision.update();
        return collision.checkSelfCollisions();
    };



    XBot::Cartesian::Planning::ValidityPredicateAggregate valid;
    valid.add(std::bind(&XBot::Cartesian::Planning::ConvexHullStability::checkStability, ch.get()), "convex_hull");
    valid.add(check_collision, "self_collisions", false);

    goal_sampler = std::make_shared<XBot::Cartesian::Planning::GoalSamplerBase>(solver);
    goal_sampler->setValidityCheker(std::bind(&XBot::Cartesian::Planning::ValidityPredicateAggregate::checkAll, &valid, nullptr));

    ros::Rate rate(100);
    while(ros::ok())
    {
        ros_server->run();
        ch_viz->publish();

        ros::spinOnce();
        rate.sleep();
    }

}
