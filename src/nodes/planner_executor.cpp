#include "planner_executor.h"

#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"
#include "validity_checker/validity_predicate_aggregate.h"

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/LoadObject.hpp>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

using namespace XBot::Cartesian;

PlannerExecutor::PlannerExecutor():
    _nh("planner"),
    _nhpr("~")
{
    init_load_model();
    init_load_config();
    init_load_planner();
    init_load_validity_checker();
}

void PlannerExecutor::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);
    _model->setJointPosition(qhome);
    _model->update();
}

void PlannerExecutor::init_load_config()
{
    if(!_nhpr.hasParam("planner_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'planner_config' missing");
    }

    // load planner config file (yaml)
    std::string planner_config_string;
    _nhpr.getParam("planner_config", planner_config_string);

    _planner_config = YAML::Load(planner_config_string);
}

void PlannerExecutor::init_load_planner()
{
    // manifold variable
    ompl::base::ConstraintPtr ompl_constraint;

    // if a manifold was specified, load it from param server
    if(_planner_config["manifold"])
    {
        YAML_PARSE_OPTION(_planner_config["manifold"],
                param_name,
                std::string,
                "constraint_problem_description");

        std::string problem_description_string;
        YAML::Node ik_yaml_constraint;

        if(!_nh.hasParam(param_name) ||
                !_nh.getParam(param_name,
                              problem_description_string))
        {
            throw std::runtime_error("problem_description '" + param_name + "' parameter missing");
        }
        else
        {
            ik_yaml_constraint = YAML::Load(problem_description_string);
        }

        auto ik_prob_constraint = ProblemDescription(ik_yaml_constraint, _model);

        CartesianInterfaceImpl::Ptr constraint_ci = Utils::LoadObject<CartesianInterfaceImpl>("libCartesianOpenSot.so",
                                                                                              "create_instance",
                                                                                              _model,
                                                                                              ik_prob_constraint);


        auto ik_solver = std::make_shared<Planning::PositionCartesianSolver>(constraint_ci,
                                                                             ik_prob_constraint);

        _manifold = std::make_shared<Planning::CartesianConstraint>(ik_solver);

        ompl_constraint = _manifold;

    }

    // get state bounds
    Eigen::VectorXd qmin, qmax;
    _model->getJointLimits(qmin, qmax);

    if(_model->isFloatingBase())
    {
        qmax.head<6>() << 1, 1, 1, M_PI, M_PI, M_PI;
        qmin.head<6>() << -qmin.head<6>();

        YAML_PARSE_OPTION(_planner_config["state_space"]["floating_base_pos_min"],
                floating_base_pos_min,
                std::vector<double>,
                {});

        if(floating_base_pos_min.size() > 0)
        {
            qmin.head<3>() << floating_base_pos_min[0],
                    floating_base_pos_min[1],
                    floating_base_pos_min[2];
        }

        YAML_PARSE_OPTION(_planner_config["state_space"]["floating_base_pos_max"],
                floating_base_pos_max,
                std::vector<double>,
                {});

        if(floating_base_pos_max.size() > 0)
        {
            qmax.head<3>() << floating_base_pos_max[0],
                    floating_base_pos_max[1],
                    floating_base_pos_max[2];
        }

    }

    if(ompl_constraint)
    {
        std::cout << "Constructing a constrained ompl planner" << std::endl;
        _planner = std::make_shared<Planning::OmplPlanner>(qmin,
                                                           qmax,
                                                           ompl_constraint,
                                                           _planner_config);
    }
    else
    {
        std::cout << "Constructing an unconstrained ompl planner" << std::endl;
        _planner = std::make_shared<Planning::OmplPlanner>(qmin,
                                                           qmax,
                                                           _planner_config);
    }


}

void PlannerExecutor::init_load_validity_checker()
{
    if(!_planner_config["state_validity_check"])
    {
        std::cout << "No state validity checkers were defined" << std::endl;
        return;
    }

    Planning::ValidityPredicateAggregate vc_aggregate;

    for(auto vc : _planner_config["state_validity_check"])
    {
        auto vc_name = vc.as<std::string>();

        if(!_planner_config[vc_name])
        {
            throw std::runtime_error("Node for state validity checker '" + vc_name + "' must be defined");
        }

        auto vc_fun = Planning::MakeValidityChecker(_planner_config[vc_name],
                                                    _model,
                                                    "");

        vc_aggregate.add(vc_fun, "vc_name");

    }

    auto validity_predicate = [&vc_aggregate, this](const Eigen::VectorXd& q)
    {
        _model->setJointPosition(q);
        _model->update();
        return vc_aggregate.checkAll();
    };

    _planner->setStateValidityPredicate(validity_predicate);
}

void PlannerExecutor::init_subscribe_start_goal()
{
    _start_sub = _nh.subscribe("start/joint_states", 1,
                               &PlannerExecutor::on_start_state_recv, this);

    _goal_sub = _nh.subscribe("stop/joint_states", 1,
                              &PlannerExecutor::on_goal_state_recv, this);
}

void PlannerExecutor::on_start_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{

}

void PlannerExecutor::on_goal_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{

}
