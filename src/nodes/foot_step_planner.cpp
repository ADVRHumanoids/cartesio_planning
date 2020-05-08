#include "foot_step_planner.h"

#include "utils/parse_yaml_utils.h"


using namespace XBot::Cartesian;

FootStepPlanner::FootStepPlanner ():
    _nh("planner"),
    _nhpr("~")
{
    _sw = std::make_shared<Planning::StateWrapper>(Planning::StateWrapper::StateSpaceType::SE2SPACE, 2);
    init_load_config();
    init_load_model();
    init_load_position_cartesian_solver();
    init_load_planner();
    init_load_state_propagator();
    init_load_validity_checker(); 
    init_load_goal_generator();
    init_load_problem_definition();
    init_subscribe_start_goal();
    init_planner_srv();
    init_trajectory_publisher();
}

void FootStepPlanner::run()
{
    auto time = ros::Time::now();
    ros::spinOnce();

    publish_tf(time);
}

void FootStepPlanner::init_load_config()
{
    if(!_nhpr.hasParam("planner_config"))
    {
        throw std::runtime_error("Mandatory private parameter 'planner_config' missing");
    }

    // load planner config file (yaml)
    std::string planner_config_string;
    _nhpr.getParam("planner_config", planner_config_string);
    std::cout << "Loaded config file: \n" << planner_config_string << std::endl;

    _planner_config = YAML::Load(planner_config_string);
    
}

void FootStepPlanner::init_load_model ()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);
    _start_model = XBot::ModelInterface::getModel(cfg);
    _goal_model = XBot::ModelInterface::getModel(cfg);
    _solver_model = XBot::ModelInterface::getModel(cfg);
    
    _model->getRobotState("home", _qhome);
    _model->setJointPosition(_qhome);
    _model->update();
    
    // UNCOMMENT THIS WHEN PLANNING WITH CENTAURO: computes initial z-axis position 
    // of the wheel that will be used during the whole planner
//     Eigen::Affine3d T;
//     _model->getPose("wheel_1", T);
//     _z_wheel = T.translation().z();
    
    // Define start model
    _start_model->setJointPosition(_qhome);
    _start_model->update();
    
    // Define goal model
    _goal_model->setJointPosition(_qhome);
    _goal_model->update();
    
    // Define an extra model used by the solver
    _solver_model->setJointPosition(_qhome);
    _solver_model->update();
    
    _map = std::make_shared<XBot::Converter::MapConverter>(_nhpr, "projected_map");

}

void FootStepPlanner::init_load_position_cartesian_solver() 
{
    // Solver
    std::string problem_description;
    if(!_nh.getParam("problem_description", problem_description))
    {
        ROS_ERROR("planner/problem_description!");
        throw std::runtime_error("planner/problem_description!");
    }

    auto ik_yaml_goal = YAML::Load(problem_description);

    double ci_period = 0.1;
    auto ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), _model);
    auto ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    _ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx); 
    _solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(_ci);
    
    // Goal Solver
    problem_description;
    if(!_nh.getParam("problem_goal_description", problem_description))
    {
        ROS_ERROR("planner/problem_goal_description!");
        throw std::runtime_error("planner/problem_goal_description!");
    }

    ik_yaml_goal = YAML::Load(problem_description);

    ci_period = 0.1;
    ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), _solver_model);
    ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx); 
    _goal_solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci);
    
    // Eventually modify the postural task
    _postural_pub = _nh.advertise<sensor_msgs::JointState>("goal_sampler/Postural/reference", 10, true);
    
    std::vector<double> qdes(_model->getJointNum(), 0);
    _msg.position = qdes;
    
//     auto jmap = _model->getUrdf().joints_;
//     for (auto x:jmap)
//         _msg.name.push_back(x.first);
    
    _msg.header.stamp = ros::Time::now();
    
}


void FootStepPlanner::init_load_planner() 
{
    // Create the state Compount State Space containing the feet State Spaces and assign the relative bounds
    if (!_planner_config["state_space"]["ee_number"])
    {
        throw std::runtime_error("Mandatory 'feet_number' private parameter in 'planner_config' missing");
    }
    YAML_PARSE_OPTION(_planner_config["state_space"], ee_number, int, 2);
    YAML_PARSE_OPTION(_planner_config["state_space"], end_effector, std::vector<std::string>, {});
    YAML_PARSE_OPTION(_planner_config["state_space"], bounds_x, std::vector<double>, {});
    YAML_PARSE_OPTION(_planner_config["state_space"], bounds_y, std::vector<double>, {});
    
    _ee_number = ee_number;
    _ee_name = end_effector;
    
    ompl::base::RealVectorBounds bounds(2);
    if (bounds_x.size() > 0 && bounds_y.size() > 0)
    {       
        bounds.setLow(0, bounds_x[0]);
        bounds.setLow(1, bounds_y[0]);
        bounds.setHigh(0, bounds_x[1]);
        bounds.setHigh(1, bounds_y[1]);
    }   
    
//     std::vector<std::shared_ptr<ompl::base::RealVectorStateSpace>> spaces (_ee_number);
    std::vector<std::shared_ptr<ompl::base::SE2StateSpace>> spaces (_ee_number);
    _space = std::make_shared<ompl::base::CompoundStateSpace>();
    
    for (int i = 0; i < _ee_number; i++)
    {        
//         spaces[i] = std::make_shared<ompl::base::RealVectorStateSpace>(2);
        spaces[i] = std::make_shared<ompl::base::SE2StateSpace>();
        spaces[i]->setBounds(bounds);
        _space->addSubspace(spaces[i], 1.0);
    }
        
    // Create the Compound Control Space
    auto foot_selector = std::make_shared<ompl::control::DiscreteControlSpace>(_space, 1, _ee_number);
    foot_selector->setControlSamplerAllocator(getSampler);
    
    auto step_size = std::make_shared<ompl::control::RealVectorControlSpace>(_space, 3);
    
    YAML_PARSE_OPTION(_planner_config["control_space"], step_size_max, double, 1.0)
    ompl::base::RealVectorBounds step_bounds(3);
    step_bounds.setLow(-1*step_size_max);
    step_bounds.setHigh(step_size_max);
    
    step_size->setBounds(step_bounds);
    
    _cspace = std::make_shared<ompl::control::CompoundControlSpace>(_space);
    _cspace->addSubspace(foot_selector);
    _cspace->addSubspace(step_size);
    
    // Create an instance to the space information
    _space_info = std::make_shared<ompl::control::SpaceInformation>(_space, _cspace);  
    
    YAML_PARSE_OPTION(_planner_config["control_space"], min_control_duration, int, 1);
    YAML_PARSE_OPTION(_planner_config["control_space"], max_control_duration, int, 10);
    
    _space_info->setMinMaxControlDuration(min_control_duration, max_control_duration);
}

void FootStepPlanner::init_load_state_propagator()
{
    if (!_planner_config["propagator"])
        throw std::runtime_error("Mandatory private paramenter 'propagator' missing");
    
    if (!_planner_config["propagator"]["type"])
        std::cout << "Propagator type not set, using the default 'stepPropagator'" << std::endl;
    YAML_PARSE_OPTION(_planner_config["propagator"], type, std::string, "stepPropagator");
    
    if (!_planner_config["propagator"]["duration"])
        std::cout << "Duration not set, using default value 0.1" << std::endl;
    YAML_PARSE_OPTION(_planner_config["propagator"], duration, double, 0.1);
    
    _propagator = make_propagator(type);
    _space_info->setStatePropagator(_propagator);
    
    _space_info->setPropagationStepSize(duration);
}

void FootStepPlanner::init_load_validity_checker() 
{
    _vc_context = Planning::ValidityCheckContext(_planner_config,
                                                 _model, _nh);
    
    _vc_context.planning_scene->startMonitor();
    _vc_context.planning_scene->startMonitor();

    _get_planning_scene_srv = _nh.advertiseService("get_planning_scene",
                                                   &FootStepPlanner::get_planning_scene_service, this);

    _apply_planning_scene_srv = _nh.advertiseService("apply_planning_scene",
                                                     &FootStepPlanner::apply_planning_scene_service, this);

    auto validity_predicate = [this](const Eigen::VectorXd& q)
    {
        _model->setJointPosition(q);
        _model->update();
        return _vc_context.vc_aggregate.checkAll();
    };

    setStateValidityPredicate(validity_predicate);
}

void FootStepPlanner::setStateValidityPredicate(StateValidityPredicate svp)
{
    YAML_PARSE_OPTION(_planner_config["distance"], front_rear_x_distance, double, 0.3);
    YAML_PARSE_OPTION(_planner_config["distance"], left_right_y_distance, double, 0.8);
    YAML_PARSE_OPTION(_planner_config["distance"], max_x_distance, double, 1.0);
    YAML_PARSE_OPTION(_planner_config["distance"], max_y_distance, double, 1.0);

    auto ompl_svc = [front_rear_x_distance, left_right_y_distance, max_x_distance, max_y_distance, svp, this](const ompl::base::State * state)
    {
        std::vector<Eigen::Vector3d> ee(_ee_number);   // use when planning in SE2
//         std::vector<Eigen::VectorXd> ee(_ee_number);
        Eigen::Vector3d x_com = {0, 0, 0};
        Eigen::VectorXd x;
        Eigen::Affine3d T;
        
        // Transform state in Eigen::VectorXd and fill the IK solver
        for (int i = 0; i < _ee_number; i++)
        {
//             _sw->getState(state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), ee[i]);
            auto s = state->as<ompl::base::CompoundStateSpace::StateType>();
            ee[i](0) = s->as<ompl::base::SE2StateSpace::StateType>(i)->getX();
            ee[i](1) = s->as<ompl::base::SE2StateSpace::StateType>(i)->getY();
            ee[i](2) = s->as<ompl::base::SE2StateSpace::StateType>(i)->getYaw();
            x_com(0) += ee[i](0);
            x_com(1) += ee[i](1);
            T.translation() << ee[i](0), ee[i](1), 0 /*_z_wheel*/;
            T.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            T.rotate( Eigen::AngleAxis<double>( ee[i](2), Eigen::Vector3d(0,0,1) ));         // UNCOMMENT THIS WHEN PLANNING IN SE2

            _solver->setDesiredPose(_ee_name[i], T);
        }
//         for (int i = 0; i < _ee_number; i++)
//         {
//             std::cout << "Input State " << i << ": " << ee[i] << std::endl;
//         }
        
        

        // VALIDITY FUNCTIONS FOR CENTAURO
        // Check on relative distance
//         double x_diff; 
//         double y_diff;        
//         
//         
//         for (int i = 0; i < _ee_number; i += 2)
//         {
//             x_diff = sqrt((ee[i](0) - ee[i+1](0)) * (ee[i](0) - ee[i+1](0)) + (ee[i](1) - ee[i+1](1)) * (ee[i](1) - ee[i+1](1)));
//             y_diff = sqrt((ee[i](1) - ee[i+1](1)) * (ee[i](1) - ee[i+1](1)));
//             if (x_diff > 0.8)
//             {
//                 return false;
//             }
//         }
//         
//         // Check feet crossing
//         double xRel_w;
//         double yRel_w;
//         
//         for (int i = 0; i < _ee_number; i += 2)
//         {
//             xRel_w = ee[i](0) - ee[i+1](0);
//             yRel_w = ee[i](1) - ee[i+1](1);
//             if (yRel_w < 0.15)          
//             {
//                 return false;        
//             }
//         }
//         
//         // Check distance between front and rear feet on x-axis
//         const auto x_left = sqrt((ee[0](0) - ee[2](0)) * (ee[0](0) - ee[2](0)));
//         const auto x_right = sqrt((ee[1](0) - ee[3](0)) * (ee[1](0) - ee[3](0)));
//         const auto x_left_right = sqrt((ee[0](0) - ee[3](0)) * (ee[0](0) - ee[3](0)));
//         const auto x_right_left = sqrt((ee[1](0) - ee[2](0)) * (ee[1](0) - ee[2](0)));
//         
//         if (x_right < front_rear_x_distance || x_left < front_rear_x_distance || x_right > 0.9 || x_left > 0.9 || x_left_right < front_rear_x_distance || x_right_left < front_rear_x_distance || x_right_left > 0.8 || x_left_right > 0.8)
//         {
//             return false;
//         }
//         
//         // Check distance between front and rear feet on y-axis
//         const auto y_front = sqrt((ee[0](1) - ee[1](1)) * (ee[0](1) - ee[1](1)));
//         const auto y_rear = sqrt((ee[2](1) - ee[3](1)) * (ee[2](1) - ee[3](1)));
//         const auto y_front_rear = sqrt((ee[0](1) - ee[3](1)) * (ee[0](1) - ee[3](1)));
//         const auto y_rear_front = sqrt((ee[2](1) - ee[1](1)) * (ee[2](1) - ee[1](1)));        
//         
//         if (y_front < left_right_y_distance || y_rear < left_right_y_distance || y_front > 0.8 || y_rear > 0.8 || y_front_rear < left_right_y_distance || y_front_rear > 0.8 || y_rear_front < left_right_y_distance || y_rear_front > 0.8)
//         {
//             return false;
//         }

        
        // VALIDITY FUNCTIONS FOR COMANPLUS
        // Check whether one of the two feet is in collision with the environment
        _map->convert();
        
        for (int i = 0; i < _ee_number; i++)
        {
            Eigen::VectorXd x_occ, y_occ;
            _map->getOccupiedPoints(x_occ, y_occ);
            std::cout << "Size of occupied points: " << x_occ.size() << std::endl;
            if (_map->checkForCollision(ee[i], 0.2))
                return false;
        }
            
        
        // Check for relative distance
        double x_diff = sqrt((ee[0](0) - ee[1](0)) * (ee[0](0) - ee[1](0))); 
        double y_diff = sqrt((ee[0](1) - ee[1](1)) * (ee[0](1) - ee[1](1)));

        if (x_diff > max_x_distance || y_diff > max_y_distance)
        {
            return false;
        }
        
        // Check for relative orientation
        double res1 = (ee[0](2) - ee[1](2));
        double res2 = (boost::math::constants::pi<double>()*2 - (ee[0](2) - ee[1](2))); 
    
        if (std::min<double>(sqrt(res1*res1), sqrt(res2*res2)) > boost::math::constants::pi<double>()/6)
            return false;
        
        // Set Com Pose
        x_com(0) = x_com(0)/_ee_number;
        x_com(1) = x_com(1)/_ee_number;
        T.translation() << x_com(0), x_com(1), 0;
                
        _solver->setDesiredPose("Com", T);
        
        
        // Check for feet crossing
        double xRel_w = ee[0](0) - ee[1](0);
        double yRel_w = ee[0](1) - ee[1](1);
   
        if (-xRel_w * sin(ee[1](2)) + yRel_w * cos(ee[1](2)) > -0.19)                
        {
            return false;        
        }
        
        _model->setJointPosition(_qhome);
        _model->update();
        
        if (_solver->solve())
        {
            _solver->getModel()->getJointPosition(x);
            if (!svp(x))
            {
                Eigen::VectorXd qrand;
                std::cout << "Pose found is in collision; computing a new one..." << std::endl;
//                 if(_goal_generator->sample(qrand, 5))
//                 {
//                     std::cout << "Feasible pose found!" << std::endl;
//                     return true;
//                 }
//                 else
//                 {
//                     std::cout << "No feasible pose found, discarding state..." << std::endl;
//                     return false;
//                 }
                return false;
            }
            else
            {
                return true;
            }
        }
        else
        {
            std::cout << "IK solution not found, discarding state..." << std::endl;
            return false;
        }       
    };
    
    _space_info->setStateValidityChecker(ompl_svc);
}

void FootStepPlanner::init_load_goal_generator()
{
    _goal_generator = std::make_shared<GoalGenerator>(_ci, _vc_context);
}


void FootStepPlanner::init_load_problem_definition() 
{
    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);
    
    setStartAndGoalState();
}

void FootStepPlanner::init_subscribe_start_goal()
{   
    //TODO Wrong topic, modify for each end effector
    _start_sub = _nh.subscribe("start/joint_states", 1,
                               &FootStepPlanner::on_start_state_recv, this);

    _goal_sub = _nh.subscribe("goal/joint_states", 1,
                               &FootStepPlanner::on_goal_state_recv, this);

    _start_viz = std::make_shared<Planning::RobotViz>(_start_model,
                                                      "start/robot_markers",
                                                      _nh);
    _start_viz->setPrefix("planner/start/");

    _goal_viz = std::make_shared<Planning::RobotViz>(_goal_model,
                                                     "goal/robot_markers",
                                                     _nh);
    _goal_viz->setPrefix("planner/goal/");
}

void FootStepPlanner::setStartAndGoalState() 
{
    Eigen::Affine3d T;
    ompl::base::ScopedState<> start(_space);
    int j = 0;
    
    for (int i = 0; i < _ee_number; i++)
    {
        _start_model->getPose(_ee_name[i], T);
        start[j] = T.translation().x();
        start[j+1] = T.translation().y();
        j += _space->getSubspace(i)->getDimension();
    }
    
    // TODO interactive marker
//     urdf::Model urdf;
//     urdf.initParam("/robot_description");
//     XBot::Cartesian::CartesianMarker marker("RightFoot",
//                                             urdf,
//                                             visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D,
//                                             _nh,
//                                             "",
//                                             true);
//     
//     
    
    // COMANPLUS GOAL
    ompl::base::ScopedState<> goal(_space);
    goal[0] = goal[3] = 2.5;
    goal[1] = start[1];
    goal[4] = start[4];
    goal[2] = start[2];
    goal[5] = start[5];
    
    T.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T.translation() << goal[0], goal[1], 0;
    T.rotate( Eigen::AngleAxis<double>(goal[2], Eigen::Vector3d(0,0,1)));
    _goal_solver->setDesiredPose("r_sole", T);
    
    T.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T.translation() << goal[3], goal[4], 0;
    T.rotate( Eigen::AngleAxis<double>(goal[5], Eigen::Vector3d(0,0,1)));
    _goal_solver->setDesiredPose("l_sole", T);
    
    _goal_solver->solve();

    // CENTAURO GOAL
//     ompl::base::ScopedState<> goal(_space);
//     goal = start;
//     goal[0] += 3.0;
//     goal[2] += 3.0;
//     goal[4] += 3.0;
//     goal[6] += 3.0;
//     
//     T.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//     T.translation() << goal[0], goal[1], _z_wheel;
//     _goal_solver->setDesiredPose(_ee_name[0], T);
//     
//     T.translation() << goal[2], goal[3], _z_wheel;
//     _goal_solver->setDesiredPose(_ee_name[1], T);
//     
//     T.translation() << goal[4], goal[5], _z_wheel;
//     _goal_solver->setDesiredPose(_ee_name[2], T);
//     
//     T.translation() << goal[6], goal[7], _z_wheel;
//     _goal_solver->setDesiredPose(_ee_name[3], T);
//     
//     _goal_solver->solve();
   
    Eigen::VectorXd q;
   
    _goal_solver->getModel()->getJointPosition(q);
    _goal_model->setJointPosition(q);
    _goal_model->update();
    
    // Reset models to home state
//     _model->setJointPosition(_qhome);
//     _model->update();
//     
//     _solver_model->getJointPosition(_qhome);
//     _solver_model->update();
    
    // Set start and goal states
    _pdef->setStartAndGoalStates(start, goal, 0.5);
}

void FootStepPlanner::init_planner_srv()
{
    _planner_srv = _nh.advertiseService("compute_plan", &FootStepPlanner::planner_service, this);
}

bool FootStepPlanner::planner_service ( cartesio_planning::FootStepPlanner::Request& req, 
                                        cartesio_planning::FootStepPlanner::Response& res )
{
    if(req.time <= 0)
    {
        res.status.msg.data = "time arg should be > 0";
        res.status.val = ompl::base::PlannerStatus::ABORT;
        return true;
    }

    if(req.planner_type == "")
    {
        req.planner_type = "RRT";
    }

    std::cout << "Requested planner " << req.planner_type << std::endl;
    
    _planner = make_planner(req.planner_type);
    
    _planner->setProblemDefinition(_pdef);
    
    _planner->setup();
    
    // Print settings
    _space_info->printSettings(std::cout);
    _pdef->print(std::cout);
    
    res.status.val = callPlanner(req.time, req.planner_type);   
    
    if (res.status.val)
    {
        _path = _pdef->getSolutionPath();
        _path->print(std::cout);
        Eigen::VectorXd q_temp;
        Eigen::Affine3d T;
        std::vector<Eigen::Vector3d> ee(_ee_number);  // use when planning in SE2
//         std::vector<Eigen::VectorXd> ee(_ee_number);
        
        trajectory_msgs::JointTrajectory trj;
        int iter = 0;
             
        for (int i = 0; i < _path->as<ompl::geometric::PathGeometric>()->getStateCount(); i++)
        {
            for (int j = 0; j < _ee_number; j++)
            {
//                 _sw->getState(_path->as<ompl::geometric::PathGeometric>()->getState(i)->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(j), ee[j]);
                auto s = _path->as<ompl::geometric::PathGeometric>()->getState(i)->as<ompl::base::CompoundStateSpace::StateType>();
                ee[j](0) = s->as<ompl::base::SE2StateSpace::StateType>(j)->getX();
                ee[j](1) = s->as<ompl::base::SE2StateSpace::StateType>(j)->getY();
                ee[j](2) = s->as<ompl::base::SE2StateSpace::StateType>(j)->getYaw();
                T.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
                T.rotate( Eigen::AngleAxis<double>( ee[j](2), Eigen::Vector3d(0,0,1) ));         // UNCOMMENT THIS WHEN PLANNING IN SE2
                T.translation() << ee[j](0), ee[j](1), 0 /*_z_wheel*/;
                _solver->setDesiredPose(_ee_name[j], T);
            }
            double sum_x = 0;
            double sum_y = 0;
            for (int j = 0; j < _ee_number; j++)
            {
                sum_x += ee[j](0);
                sum_y += ee[j](1);
            }
            T.translation() << sum_x/_ee_number, sum_y/_ee_number, 0;
            _solver->setDesiredPose("Com", T);
            _solver->solve();
            
            bool pose_valid = check_state_valid(_solver->getModel());           

            // If IK solution is in collision/self-collision, find a new pose using goal_generator
            if (!pose_valid)
            {
                std::cout << "Pose found for state " << i << " does not respect constraints, finding a new IK solution" << std::endl;
                
                Eigen::VectorXd qrand;
                
                int max_iterations;
                if(_nhpr.getParam("goal_generator_max_iterations", max_iterations))
                {
                    _goal_generator->setMaxIterations(max_iterations);
                }

                double error_tolerance;
                if(_nhpr.getParam("goal_generator_error_tolerance", error_tolerance))
                {
                    _goal_generator->setErrorTolerance(error_tolerance);
                }
                             
                _goal_generator->sample(qrand, 20);   
                iter ++;
            }
            _solver->getModel()->getJointPosition(q_temp);
            _q_vect.push_back(q_temp);            
        }
        
        std::cout << "Goal generator called " << iter << " times relative to " << _path->as<ompl::geometric::PathGeometric>()->getStateCount() << " states" << std::endl;
        
        auto t = ros::Duration(0.);
        
        for(auto x : _q_vect)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(x.data(), x.data() + x.size());
            point.time_from_start = t;
            trj.points.push_back(point);
            t += ros::Duration(1.0);
        }
        
        _trj_publisher.publish(trj);
    }
}


void FootStepPlanner::init_trajectory_publisher() 
{
    _trj_publisher = _nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);
}


int FootStepPlanner::callPlanner(const double time,
                                 const std::string& planner_type)
{
    if(time <= 0.)
        return ompl::base::PlannerStatus::ABORT;

    // check start and goal state correctness
    ROS_INFO("Checking start state validity..");
    if(!check_state_valid(_start_model))
    {
        throw std::runtime_error("Invalid start state");
    }

    ROS_INFO("Checking goal state validity..");
    if(!check_state_valid(_goal_model))
    {
        throw std::runtime_error("Invalid goal state");
    }

    ROS_INFO("start and goal states are valid");

    Eigen::VectorXd qstart, qgoal;
    _start_model->getJointPosition(qstart);
    _goal_model->getJointPosition(qgoal);

    ROS_INFO("Enforcing bounds...");
    enforce_bounds(qstart);
    enforce_bounds(qgoal);
    std::cout<<"...done!"<<std::endl;
    
    _status = _planner->solve(time);

    return ompl::base::PlannerStatus::StatusType(_status);
}

bool FootStepPlanner::check_state_valid(XBot::ModelInterface::ConstPtr model)
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
    _model->getJointLimits(qmin, qmax);
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
    return valid;
}

void FootStepPlanner::enforce_bounds(Eigen::VectorXd & q) const
{
    Eigen::VectorXd qmin, qmax;
    _model->getJointLimits(qmin, qmax);

    q = q.cwiseMin(qmax).cwiseMax(qmin);
}


ompl::control::ControlSamplerPtr FootStepPlanner::getSampler ( const ompl::control::ControlSpace* cspace ) 
{
    return std::make_shared<XBot::Cartesian::Planning::stepSampler>(cspace);
}

ompl::control::StatePropagatorPtr FootStepPlanner::make_propagator ( const std::__cxx11::string propagatorType ) 
{
    if (propagatorType == "stepPropagator")
        return std::make_shared<Planning::Propagators::stepPropagator>(_space_info, _sw);
    
    else if (propagatorType == "stepPropagatorSE2")
        return std::make_shared<Planning::Propagators::stepPropagatorSE2>(_space_info, _sw);
}

ompl::base::PlannerPtr FootStepPlanner::make_planner ( std::__cxx11::string plannerType ) 
{
    if (plannerType == "RRT")
        return std::make_shared<ompl::control::RRT>(_space_info);
    
    else if (plannerType == "SST")
        return std::make_shared<ompl::control::SST>(_space_info);
    
    else if (plannerType == "KPIECE")
        return std::make_shared<ompl::control::KPIECE1>(_space_info);
    
    else 
        std::runtime_error("Selected planner does not exist or is not available");
}


bool FootStepPlanner::get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req,
                                                 moveit_msgs::GetPlanningScene::Response& res)
{
    _vc_context.planning_scene->getPlanningScene(req, res);
    return true;
}

bool FootStepPlanner::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request & req,
                                                   moveit_msgs::ApplyPlanningScene::Response & res)
{
    _vc_context.planning_scene->applyPlanningScene(req.scene);
    return true;
}

void FootStepPlanner::publish_tf(ros::Time time)
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
    
    // Publish desired postural pose
    _postural_pub.publish(_msg);

}

void FootStepPlanner::on_start_state_recv(const sensor_msgs::JointStateConstPtr & msg)
{
    // tbd: input checking

    XBot::JointNameMap q;
    for(int i = 0; i < msg->name.size(); i++)
    {
        q[msg->name[i]] = msg->position[i];
    }

    _start_model->setJointPosition(q);
    _start_model->update();
}

void FootStepPlanner::on_goal_state_recv(const sensor_msgs::JointStateConstPtr & msg)
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



