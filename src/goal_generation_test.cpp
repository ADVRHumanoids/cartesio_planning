#include "ik/position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>
#include <validity_checker/stability/stability_detection.h>
#include <validity_checker/validity_predicate_aggregate.h>
#include <functional>
#include <validity_checker/collisions/planning_scene_wrapper.h>
#include <nodes/goal_generation.h>

#include "cartesio_planning/CartesioGoal.h"

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "position_ik_test");
    ros::NodeHandle n("~");
    ros::NodeHandle nh("planner");

    // obtain robot model from param server
    auto cfg = LoadOptions(LoadFrom::PARAM);
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

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

    if(!n.hasParam("planner_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'planner_config' missing");
    }

    // load planner config file (yaml)
    std::string planner_config_string;
    n.getParam("planner_config", planner_config_string);

    XBot::Cartesian::Planning::ValidityCheckContext vc_context(YAML::Load(planner_config_string),
                                                               model, nh);

    GoalGenerator goal_gen(ci, vc_context, nh);



    ros::Rate rate(100);
    while(ros::ok())
    {
        goal_gen.update();

        ros::spinOnce();
        rate.sleep();
    }

}
