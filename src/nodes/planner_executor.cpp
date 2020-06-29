#include "planner_executor.h"

#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"

#include <trajectory_msgs/JointTrajectory.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

#include <cartesian_interface/utils/LoadObject.hpp>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "cartesio_planning/CartesianTrajectory.h"

#include <bprinter/table_printer.h>

using namespace XBot::Cartesian;

PlannerExecutor::PlannerExecutor():
    _nh("planner"),
    _nhpr("~")
{
    init_load_config();
    init_load_model();
    init_load_planner();
    init_load_validity_checker();
    init_goal_generator();
    init_subscribe_start_goal();
    init_trj_publisiher();
    init_planner_srv();
    init_interpolator();
}

void PlannerExecutor::run()
{
    auto time = ros::Time::now();
    ros::spinOnce();

    if(_use_goal_generator)
        _goal_generator->update();

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

    std::string world_frame_link;
    if(_nhpr.hasParam("world_frame_link"))
    {
        _nhpr.getParam("world_frame_link", world_frame_link);
        Eigen::Affine3d T;
        if(_model->getPose(world_frame_link,T))
        {
            ROS_INFO("Setting planner world frame in %s", world_frame_link.c_str());

            _model->setFloatingBasePose(T.inverse());
            _model->update();

            _start_model->setFloatingBasePose(T.inverse());
            _start_model->update();

            _goal_model->setFloatingBasePose(T.inverse());
            _goal_model->update();
        }
        else
            ROS_ERROR("world_frame_link %s does not exists, keeping original world!", world_frame_link.c_str());
    }
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

        _manifold = make_manifold(problem_description_string);

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

    _start_srv = _nh.advertiseService("start_pose/joint_states", &PlannerExecutor::start_service, this);
    _goal_srv = _nh.advertiseService("goal_pose/joint_states", &PlannerExecutor::goal_service, this);
//    _goal_srv = _nh.advertiseService("goal/joint_states", &PlannerExecutor::goal_service, this);

    _start_sub = _nh.subscribe("start/joint_states", 1,
                               &PlannerExecutor::on_start_state_recv, this);

    if(!_use_goal_generator)
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
    if(_nhpr.hasParam("distal_links"))
    {
        XmlRpc::XmlRpcValue v;
        _nhpr.getParam("distal_links", v);
        for(int i =0; i < v.size(); i++)
            _distal_links.push_back(v[i]);

        if(_nhpr.hasParam("base_links"))
        {
            XmlRpc::XmlRpcValue v;
            _nhpr.getParam("base_links", v);
            for(int i =0; i < v.size(); i++)
                _base_links.push_back(v[i]);
        }
        else
        {
            for(unsigned int i = 0; i < _distal_links.size(); ++i)
                _base_links.push_back("world");
        }


        if(_base_links.size() != _distal_links.size())
        {
            throw std::runtime_error("base_links and distal_links params should have same size!");
        }

        for(unsigned int i = 0; i < _distal_links.size(); ++i)
            _cartesian_trajectory_publishers.push_back(_nh.advertise<cartesio_planning::CartesianTrajectory>
                                                       (_distal_links[i]+"/cartesian_trajectory",1, true));
    }

    _trj_pub = _nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);
}

void PlannerExecutor::init_planner_srv()
{
    _planner_srv = _nh.advertiseService("compute_plan", &PlannerExecutor::planner_service, this);
}

void PlannerExecutor::init_goal_generator()
{
    if(!_nhpr.getParam("use_goal_generator" ,_use_goal_generator))
        _use_goal_generator = false;

    if(_use_goal_generator)
    {
        std::string problem_description_string;
        if(!_nh.getParam("problem_description_goal", problem_description_string))
        {
            ROS_ERROR("planner/problem_description_goal not provided!");
            throw std::runtime_error("planner/problem_description_goal not provided!");
        }

        auto ik_yaml_goal = YAML::Load(problem_description_string);

                double ci_period = 1.0;
                auto ci_ctx = std::make_shared<Context>(
                            std::make_shared<Parameters>(ci_period),
                            _model);

                auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);

        CartesianInterfaceImpl::Ptr ci = Utils::LoadObject<CartesianInterfaceImpl>("libCartesianOpenSot.so",
                                                                                   "create_instance",
                                                                                   _model,
                                                                                   ik_prob);


        _goal_generator = std::make_shared<GoalGenerator>(ci, _vc_context);

        int max_iterations;
        if(_nhpr.getParam("goal_generator_max_iterations", max_iterations))
            _goal_generator->setMaxIterations(max_iterations);

        double error_tolerance;
        if(_nhpr.getParam("goal_generator_error_tolerance", error_tolerance))
            _goal_generator->setErrorTolerance(error_tolerance);

        _service_goal_sampler = _nh.advertiseService("goal_sampler_service",
                                                     &PlannerExecutor::goal_sampler_service, this);

        ROS_WARN("goal generator is going to be used, disabling goal from topic");
    }
}

void PlannerExecutor::init_interpolator()
{
    _interpolator = std::make_shared<CartesianTrajectoryInterpolation>();

    ///TODO: qdot, qddot limits?
}

bool PlannerExecutor::goal_sampler_service(cartesio_planning::CartesioGoal::Request &req,
                                           cartesio_planning::CartesioGoal::Response &res)
{
    Eigen::VectorXd q;
    if(!_goal_generator->sample(q, req.time)){
        res.status.val = res.status.TIMEOUT;
        res.status.msg.data = "TIMEOUT";
    }
    else
    {
        res.status.val = res.status.EXACT_SOLUTION;
        res.status.msg.data = "EXACT_SOLUTION";

        res.sampled_goal.name = _goal_model->getEnabledJointNames();
        res.sampled_goal.position.resize(q.size());
        Eigen::VectorXd::Map(&res.sampled_goal.position[0], q.size()) = q;
        res.sampled_goal.header.stamp = ros::Time::now();
    }

    if(_manifold)
        _manifold->project(q);
    _goal_model->setJointPosition(q);
    _goal_model->update();

    return true;
}

Planning::CartesianConstraint::Ptr PlannerExecutor::make_manifold(std::string problem_description_string)
{

    auto ik_yaml_constraint = YAML::Load(problem_description_string);

    double ci_period = 1.0;
    auto ci_ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(ci_period),
                _model);

    auto ik_prob_constraint = ProblemDescription(ik_yaml_constraint, ci_ctx);

    auto constraint_ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                              ik_prob_constraint,
                                                              ci_ctx);


    auto ik_solver = std::make_shared<Planning::PositionCartesianSolver>(constraint_ci);

    return std::make_shared<Planning::CartesianConstraint>(ik_solver);
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
    const double q_tol = 1e-3;

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
    _manifold->getTolerance();


    const double err_threshold = _manifold->getTolerance();
    double err = error.cwiseAbs().maxCoeff();
    if(err > err_threshold)
    {
        valid = false;
        std::cout << "Invalid state, not on manifold (error = " << err << " > "<<err_threshold<<")" << std::endl;
    }

    return valid;
}

void PlannerExecutor::setStartState(const XBot::JointNameMap& q)
{
    _start_model->setJointPosition(q);
    _start_model->update();

    if(_manifold)
    {
        if(_model != _start_model)
        {
            _model->syncFrom(*_start_model, XBot::Sync::Position);
        }
        _manifold->reset(); // note: manifold is set according to start state
    }
}

void PlannerExecutor::on_start_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }


    setStartState(q);

}

void PlannerExecutor::setGoalState(const XBot::JointNameMap& q)
{
    _goal_model->setJointPosition(q);
    _goal_model->update();

    Eigen::VectorXd qq;
    _goal_model->getJointPosition(qq);

    if(_manifold)
        _manifold->project(qq);

    _goal_model->setJointPosition(qq);
    _goal_model->update();
}

void PlannerExecutor::on_goal_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }

    setGoalState(q);
}

bool PlannerExecutor::start_service(cartesio_planning::CartesioJointStates::Request& req,
                                    cartesio_planning::CartesioJointStates::Response& res)
{

    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < req.joint_states.name.size(); i++)
    {
        q[req.joint_states.name[i]] = req.joint_states.position[i];
    }

    setStartState(q);
    return true;

}

bool PlannerExecutor::goal_service(cartesio_planning::CartesioJointStates::Request& req,
                                    cartesio_planning::CartesioJointStates::Response& res)
{

    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < req.joint_states.name.size(); i++)
    {
        q[req.joint_states.name[i]] = req.joint_states.position[i];
    }

    setGoalState(q);
    return true;
}

bool PlannerExecutor::planner_service(cartesio_planning::CartesioPlanner::Request& req,
                                      cartesio_planning::CartesioPlanner::Response& res)
{


    if(req.time <= 0)
    {
        res.status.msg.data = "time arg should be > 0";
        res.status.val = ompl::base::PlannerStatus::ABORT;
        return true;
    }

    if(req.interpolation_time <= 0)
    {
        res.status.msg.data = "interpolation_time arg should be > 0";
        res.status.val = ompl::base::PlannerStatus::ABORT;
        return true;
    }

    /* fill cartesian frames specified in topic */
    if(req.base_links.size() > req.distal_links.size())
    {
        throw std::runtime_error("There are more base links than distal links!");
    }

    std::vector<std::string> distal_links;
    for(auto elem : req.distal_links)
    {
        distal_links.push_back(elem);
    }

    std::vector<std::string> base_links;
    if(req.base_links.empty())
    {
        res.status.msg.data = "Filling missing base links with frame 'world'.";
        for(auto elem : distal_links)
            base_links.push_back("world");
    }
    else
    {
        for(auto elem : req.base_links)
        {
            base_links.push_back(elem);
        }
    }

    if(req.planner_type == "")
    {
        req.planner_type = "RRTstar";
    }

    std::cout << "Requested planner: " << req.planner_type << std::endl;

    std::vector<Eigen::VectorXd> trajectory;
    std::vector<std::vector<Eigen::Affine3d> > cartesian_trajectories;

//    bprinter::TablePrinter tp(&std::cout);
//    tp.PrintHeader();
//    tp <<
//    tp.PrintFooter();

    if (req.trajectory_space == "Joint")
    {
        std::cout << "Joint space selected. Computing Joint trajectory." << std::endl;
        res.status.val = callPlanner(req.time, req.planner_type, req.interpolation_time, trajectory);
    }
    else if (req.trajectory_space == "")
    {
        if (_distal_links.empty() && distal_links.empty())
        {
            std::cout << "No space trajectory defined, and no cartesian frames were inserted. Computing Joint trajectory." << std::endl;
            res.status.val = callPlanner(req.time, req.planner_type, req.interpolation_time, trajectory);
        }
        else
        {
            std::vector<std::pair<std::string, std::string> > base_distal_links;
            if (!distal_links.empty())
            {
                std::cout << "No space trajectory defined, but cartesian frames were inserted. Computing Cartesian trajectory using selected frames." << std::endl;
                for(unsigned int i = 0; i < _distal_links.size(); ++i)
                    base_distal_links.push_back(std::pair<std::string, std::string>(base_links[i], distal_links[i]));
            }
            else
            {
                std::cout << "No space trajectory defined, but found cartesian frames from param. Computing Cartesian trajectory using param frames." << std::endl;
                for(unsigned int i = 0; i < _distal_links.size(); ++i)
                    base_distal_links.push_back(std::pair<std::string, std::string>(_base_links[i], _distal_links[i]));
            }
            res.status.val = callPlanner(req.time, req.planner_type, req.interpolation_time, base_distal_links,
                                         trajectory, cartesian_trajectories);

            for (auto elem :base_distal_links)
            {
                std::cout << "Base frame: " << elem.first << " -- Distal frame: " << elem.second << std::endl;
            }
        }
    }
    else if (req.trajectory_space == "Cartesian")
    {
        if (_distal_links.empty() && distal_links.empty())
        {
            std::cout << "No cartesian frames specified." << std::endl;
            res.status.val = ompl::base::PlannerStatus::ABORT;
        }
        else
        {
            std::vector<std::pair<std::string, std::string> > base_distal_links;
            if (!distal_links.empty())
            {
                std::cout << "Computing Cartesian trajectory using selected frames." << std::endl;
                for(unsigned int i = 0; i < distal_links.size(); ++i)
                    base_distal_links.push_back(std::pair<std::string, std::string>(base_links[i], distal_links[i]));
            }
            else
            {
                std::cout << "Computing Cartesian trajectory using param frames." << std::endl;
                for(unsigned int i = 0; i < _distal_links.size(); ++i)
                    base_distal_links.push_back(std::pair<std::string, std::string>(_base_links[i], _distal_links[i]));
            }
            res.status.val = callPlanner(req.time, req.planner_type, req.interpolation_time, base_distal_links,
                                         trajectory, cartesian_trajectories);

            for (auto elem :base_distal_links)
            {
                std::cout << "Base frame: " << elem.first << " -- Distal frame: " << elem.second << std::endl;
            }
        }
    }



    res.status.msg.data = _planner->getPlannerStatus().asString();

    if(res.status.val)
    {
        trajectory_msgs::JointTrajectory msg;
        msg.joint_names = _model->getEnabledJointNames();
        res.joint_trajectory.joint_names = _model->getEnabledJointNames();
        auto t = ros::Duration(0.);

        for(auto x : trajectory)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(x.data(), x.data() + x.size());
            point.time_from_start = t;
            msg.points.push_back(point);
            res.joint_trajectory.points.push_back(point);
            t += ros::Duration(req.interpolation_time);
        }

        _trj_pub.publish(msg);

        if(cartesian_trajectories.size() > 0)
        {
            res.cartesian_trajectory.resize(cartesian_trajectories.size());

            for(unsigned int i = 0; i < cartesian_trajectories.size(); ++i)
            {
                cartesio_planning::CartesianTrajectory msg;
                auto t = ros::Duration(0.);

                if (base_links.empty())
                {
                    msg.base_link = _base_links[i];
                    msg.distal_link = _distal_links[i];

                    res.cartesian_trajectory[i].base_link = _base_links[i];
                    res.cartesian_trajectory[i].distal_link = _distal_links[i];
                }
                else
                {
                    msg.base_link = base_links[i];
                    msg.distal_link = distal_links[i];

                    res.cartesian_trajectory[i].base_link = base_links[i];
                    res.cartesian_trajectory[i].distal_link = distal_links[i];
                }

                for(Eigen::Affine3d p : cartesian_trajectories[i])
                {
                    geometry_msgs::Pose pp;
                    pp.position.x = p.translation()[0]; pp.position.y = p.translation()[1]; pp.position.z = p.translation()[2];
                    Eigen::Quaterniond q(p.linear());
                    pp.orientation.x = q.x(); pp.orientation.y = q.y(); pp.orientation.z = q.z(); pp.orientation.w = q.w();
                    msg.frames.push_back(pp);
                    msg.time_from_start.push_back(t);

                    res.cartesian_trajectory[i].frames.push_back(pp);
                    res.cartesian_trajectory[i].time_from_start.push_back(t);

                    t += ros::Duration(req.interpolation_time);
                }

                _cartesian_trajectory_publishers[i].publish(msg);
            }
        }
    }

    return true;
}

int PlannerExecutor::callPlanner(const double time, const std::string& planner_type, const double interpolation_time,
                const std::vector<std::pair<std::string, std::string> > base_distal_links,
                std::vector<Eigen::VectorXd>& trajectory,
                std::vector<std::vector<Eigen::Affine3d> >& cartesian_trajectories)
{
    int ret = callPlanner(time, planner_type, interpolation_time, trajectory);

    if(_interpolator->isValid())
    {
        for(unsigned int i = 0; i < base_distal_links.size(); ++i)
        {
            std::vector<Eigen::Affine3d> cartesian_trajectory;
            double t = 0.;
            while(t <= _interpolator->getTrajectoryEndTime())
            {
                cartesian_trajectory.push_back(_interpolator->evaluate(t, base_distal_links[i].first, base_distal_links[i].second));
                t += interpolation_time;
            }
            cartesian_trajectories.push_back(cartesian_trajectory);
        }

    }

    return ret;
}

int PlannerExecutor::callPlanner(const double time, const std::string& planner_type,
                                 const double interpolation_time, std::vector<Eigen::VectorXd>& trajectory)
{
    if(time <= 0.)
        return ompl::base::PlannerStatus::ABORT;

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

    std::cout<<"start and goal states are valid"<<std::endl;

    Eigen::VectorXd qstart, qgoal;
    _start_model->getJointPosition(qstart);
    _goal_model->getJointPosition(qgoal);

    std::cout<<"Enforcing bounds..."<<std::endl;
    enforce_bounds(qstart);
    enforce_bounds(qgoal);
    std::cout<<"...done!"<<std::endl;


    _planner->setStartAndGoalStates(qstart, qgoal);

    _planner->solve(time, planner_type);


    std::vector<Eigen::VectorXd> raw_trajectory;
    if(_planner->getPlannerStatus())
        raw_trajectory = _planner->getSolutionPath();

    _interpolator->compute(raw_trajectory);
    double t = 0.;
    while(t <= _interpolator->getTrajectoryEndTime())
    {
        trajectory.push_back(_interpolator->evaluate(t));
        t += interpolation_time;
    }

    return ompl::base::PlannerStatus::StatusType(_planner->getPlannerStatus());
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
        ROS_WARN("START state is NOT valid!");
    }

    std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();

    for(unsigned int i = 0; i < red_links.size(); ++i)
        ROS_WARN("start robot: colliding link %i --> %s",i ,red_links[i].c_str());

    _start_viz->setRGBA(start_color);
    _start_viz->publishMarkers(time, red_links);

    /* Publish goal markers */
    auto goal_color = (Eigen::Vector4d() << 0.0, 1.0, 0.0, 0.5).finished();

    bool goal_valid = check_state_valid(_goal_model);

    if(!goal_valid)
    {
        goal_color << 178./255., 77./255., 0, 0.5;
        ROS_WARN("GOAL state is NOT valid!");
    }

    red_links = _vc_context.planning_scene->getCollidingLinks();

    for(unsigned int i = 0; i < red_links.size(); ++i)
        ROS_WARN("goal robot: colliding link %i --> %s",i ,red_links[i].c_str());

    _goal_viz->setRGBA(goal_color);
    _goal_viz->publishMarkers(time, red_links);

}

void PlannerExecutor::enforce_bounds(Eigen::VectorXd & q) const
{
    Eigen::VectorXd qmin, qmax;
    _planner->getBounds(qmin, qmax);

    q = q.cwiseMin(qmax).cwiseMax(qmin);
}
