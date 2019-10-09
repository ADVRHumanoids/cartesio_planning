#include "planner_executor.h"

#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/LoadObject.hpp>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

using namespace XBot::Cartesian;

PlannerExecutor::PlannerExecutor():
    _nh("planner"),
    _nhpr("~")
{
    init_load_config();
    init_load_model();
    init_load_planner();
    init_load_validity_checker();
    init_subscribe_start_goal();
    init_trj_publisiher();
    init_planner_srv();
}

void PlannerExecutor::run()
{
    auto time = ros::Time::now();
    ros::spinOnce();

    publish_tf(time);

}

void PlannerExecutor::init_load_model()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);
    _start_model = XBot::ModelInterface::getModel(cfg);
    _goal_model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    _model->getRobotState("home", qhome);

    _model->setJointPosition(qhome);
    _model->update();

    _start_model->setJointPosition(qhome);
    _start_model->update();

    _goal_model->setJointPosition(qhome);
    _goal_model->update();
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
        qmin.head<6>() << -qmax.head<6>();

        YAML_PARSE_OPTION(_planner_config["state_space"],
                floating_base_pos_min,
                std::vector<double>,
                {});

        if(floating_base_pos_min.size() > 0)
        {
            qmin.head<3>() << floating_base_pos_min[0],
                    floating_base_pos_min[1],
                    floating_base_pos_min[2];
        }

        YAML_PARSE_OPTION(_planner_config["state_space"],
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
    _vc_context = Planning::ValidityCheckContext(_planner_config,
                                                 _model, _nh);

    _vc_context.planning_scene->startMonitor();

    _vc_context.planning_scene->startMonitor();

    _get_planning_scene_srv = _nh.advertiseService("get_planning_scene",
                                                   &PlannerExecutor::get_planning_scene_service, this);

    _apply_planning_scene_srv = _nh.advertiseService("apply_planning_scene",
                                                     &PlannerExecutor::apply_planning_scene_service, this);

    auto validity_predicate = [this](const Eigen::VectorXd& q)
    {
        _model->setJointPosition(q);
        _model->update();
        return _vc_context.vc_aggregate.checkAll();
    };

    _planner->setStateValidityPredicate(validity_predicate);
}

void PlannerExecutor::init_subscribe_start_goal()
{
    _start_sub = _nh.subscribe("start/joint_states", 1,
                               &PlannerExecutor::on_start_state_recv, this);

    _goal_sub = _nh.subscribe("goal/joint_states", 1,
                              &PlannerExecutor::on_goal_state_recv, this);

    _start_viz = std::make_shared<Planning::RobotViz>(_model,
                                                     "start/robot_markers",
                                                     _nh);
    _start_viz->setPrefix("planner/start/");

    _goal_viz = std::make_shared<Planning::RobotViz>(_model,
                                                     "goal/robot_markers",
                                                     _nh);
    _goal_viz->setPrefix("planner/goal/");
}

void PlannerExecutor::init_trj_publisiher()
{
    _trj_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);
}

void PlannerExecutor::init_planner_srv()
{
    _planner_srv = _nh.advertiseService("compute_plan", &PlannerExecutor::planner_service, this);
}

bool PlannerExecutor::check_state_valid(XBot::ModelInterface::ConstPtr model)
{
    if(_model != model)
    {
        _model->syncFrom(*model, XBot::Sync::Position);
    }

    bool valid = true;

    std::vector<std::string> failed_checks;
    if(!_vc_context.vc_aggregate.checkAll(&failed_checks))
    {
        valid = false;

        std::cout << "Invalid state, failed validity checks were: \n";
        for(int i = 0; i < failed_checks.size(); i++)
        {
            std::cout << " - '" << failed_checks[i] << "'\n";
        }
        std::cout.flush();
    }

    Eigen::VectorXd q;
    Eigen::VectorXd qmin, qmax;
    _model->getJointPosition(q);
    _planner->getBounds(qmin, qmax);
    const double q_tol = 1e-6;

    if((q.array() < qmin.array() - q_tol).any() || (q.array() > qmax.array() + q_tol).any())
    {
        valid = false;
        std::cout << "Invalid state, violates bounds" << std::endl;

        for(int i = 0; i < _model->getJointNum(); i++)
        {
            if(q[i] < qmin[i] || q[i] > qmax[i])
            {
                std::cout << _model->getEnabledJointNames().at(i) <<
                         ": " << qmin[i] << " <= " << q[i] <<
                         " <= " << qmax[i] << "\n";
            }
        }
        std::cout.flush();
    }

    if(!_manifold)
    {
        return valid;
    }

    Eigen::VectorXd error(_manifold->getCoDimension());
    _manifold->function(q, error);

    const double err_threshold = 1e-6;
    double err = error.cwiseAbs().maxCoeff();
    if(err > err_threshold)
    {
        valid = false;
        std::cout << "Invalid state, not on manifold (error = " << err << ")" << std::endl;
    }

    return valid;
}

void PlannerExecutor::on_start_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }

    _start_model->setJointPosition(q);
    _start_model->update();

    if(_manifold)
    {
        _manifold->reset(); // note: manifold is set according to start state
    }

}

void PlannerExecutor::on_goal_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }

    _goal_model->setJointPosition(q);
    _goal_model->update();

}

bool PlannerExecutor::planner_service(cartesio_planning::CartesioPlanner::Request& req,
                                      cartesio_planning::CartesioPlanner::Response& res)
{
    // check start and goal state correctness
    std::cout << "Checking start state validity.." << std::endl;
    if(!check_state_valid(_start_model))
    {
        throw std::runtime_error("Invalid start state");
    }

    std::cout << "Checking goal state validity.." << std::endl;
    if(!check_state_valid(_goal_model))
    {
        throw std::runtime_error("Invalid goal state");
    }

    Eigen::VectorXd qstart, qgoal;
    _start_model->getJointPosition(qstart);
    _goal_model->getJointPosition(qgoal);

    _planner->setStartAndGoalStates(qstart, qgoal);

    if(req.time <= 0)
    {
        res.status.msg.data = "time arg should be > 0";
        res.status.val = ompl::base::PlannerStatus::ABORT;
        return true;
    }


    if(req.planner_type == "")
    {
        req.planner_type = "RRTstar";
    }

    std::cout << "Requested planner " << req.planner_type << std::endl;

    _planner->solve(req.time, req.planner_type);

    res.status.val = ompl::base::PlannerStatus::StatusType(_planner->getPlannerStatus());
    res.status.msg.data = _planner->getPlannerStatus().asString();

    if(_planner->getPlannerStatus())
    {
        trajectory_msgs::JointTrajectory msg;
        msg.joint_names = _model->getEnabledJointNames();
        auto t = ros::Duration(0);

        for(auto x : _planner->getSolutionPath())
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(x.data(), x.data() + x.size());
            t += ros::Duration(0.1);
            point.time_from_start = t;
            msg.points.push_back(point);
        }

        _trj_pub.publish(msg);
    }

    return true;
}

bool PlannerExecutor::get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req,
                                                 moveit_msgs::GetPlanningScene::Response& res)
{
    _vc_context.planning_scene->getPlanningScene(req, res);
    return true;
}

bool PlannerExecutor::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request & req,
                                                   moveit_msgs::ApplyPlanningScene::Response & res)
{
    _vc_context.planning_scene->applyPlanningScene(req.scene);
    return true;
}

void PlannerExecutor::publish_tf(ros::Time time)
{
    /* Publish start markers */
    auto start_color = (Eigen::Vector4d() << 0.0, 0.0, 1.0, 0.5).finished();

    bool start_valid = check_state_valid(_start_model);

    if(!start_valid)
    {
        start_color << 178./255., 0, 77./255., 0.5;
    }

    std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();
    _start_viz->setRGBA(start_color);
    _start_viz->publishMarkers(time, red_links);

    /* Publish goal markers */
    auto goal_color = (Eigen::Vector4d() << 0.0, 1.0, 0.0, 0.5).finished();

    bool goal_valid = check_state_valid(_goal_model);

    if(!goal_valid)
    {
        goal_color << 178./255., 77./255., 0, 0.5;
    }

    red_links = _vc_context.planning_scene->getCollidingLinks();
    _goal_viz->setRGBA(goal_color);
    _goal_viz->publishMarkers(time, red_links);

}
