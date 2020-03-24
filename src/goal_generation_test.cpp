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
    double ci_period = 0.01;
    auto ci_ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(ci_period),
                model);
    auto ik_prob = ProblemDescription(ik_yaml, ci_ctx);


    std::string impl_name = "OpenSot";
    auto ci = CartesianInterfaceImpl::MakeInstance(impl_name, ik_prob, ci_ctx);

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

    GoalGenerator goal_gen(ci, vc_context);



    ros::Rate rate(1./ci_ctx->params()->getControlPeriod());
    while(ros::ok())
    {
        goal_gen.update();

        ros::spinOnce();
        rate.sleep();
    }

}
