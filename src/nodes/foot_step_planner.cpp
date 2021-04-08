#include "foot_step_planner.h"

#include <thread>
#include <chrono>

#include "utils/parse_yaml_utils.h"


using namespace XBot::Cartesian;

FootStepPlanner::FootStepPlanner ():
    _nh("planner"),
    _nhpr("~"),
    _n(),
    _counter(0),
    _goalSampler_counter(0)
{
    if (!_n.hasParam("contact_type"))
        std::runtime_error("'contact_type' parameter missing!");
    
    std::string contact_type; 
    _n.getParam("contact_type", contact_type);
    
    if (contact_type == "point")
        _sw = std::make_shared<Planning::StateWrapper>(Planning::StateWrapper::StateSpaceType::REALVECTOR, 2);
    else if (contact_type == "surface")
        _sw = std::make_shared<Planning::StateWrapper>(Planning::StateWrapper::StateSpaceType::SE2SPACE, 3);

    _logger = XBot::MatLogger2::MakeLogger("/home/luca/src/MultiDoF-superbuild/external/cartesio_planning/log/NSPG_log");
    _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    
    init_load_config();
    init_load_model();
    init_load_position_cartesian_solver();
    init_load_planner();
    init_load_goal_service();
    init_load_state_propagator();
    init_load_validity_checker(); 
    init_load_goal_generator();
    init_load_problem_definition();
    init_planner_srv();
    init_trajectory_publisher();
    init_xbotcore_publisher();
    
}

void FootStepPlanner::run()
{
    auto time = ros::Time::now();
    _rsc->run();
    
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
    
    _n.getParam("goalSamplerType", _goalSamplerType);
    
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
    
    // Define start model
    _start_model->setJointPosition(_qhome);
    _start_model->update();
    
    // Define goal model
    _goal_model->setJointPosition(_qhome);
    _goal_model->update();
    
    // Mesh_viz
    _start_viz = std::make_shared<Planning::RobotViz>(_start_model,
                                                      "start/robot_markers",
                                                      _nh);
    _start_viz->setPrefix("planner/start/");

    _goal_viz = std::make_shared<Planning::RobotViz>(_goal_model,
                                                     "goal/robot_markers",
                                                     _nh);
    _goal_viz->setPrefix("planner/goal/");
    
    // Define an extra model used by the solver
    _solver_model->setJointPosition(_qhome);
    _solver_model->update();
    
    _map = std::make_shared<XBot::Converter::MapConverter>(_n, "projected_map");
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
    
//     Eigen::Affine3d T;
//     T.translation() << 0.35, -0.6, -0.9;
//     T.linear() = Eigen::Quaternion<double>(0,1,0,0).toRotationMatrix();
//     _solver->setDesiredPose("EEA_link", T);
//     T.translation() << 0.35, 0.6, -0.9;
//     _solver->setDesiredPose("EEB_link", T);
//     T.translation() << -0.7, 0.0, -0.9;
//     _solver->setDesiredPose("EEC_link", T);
//     
//     _solver->solve();
//     Eigen::VectorXd q(_model->getJointNum());
//     _solver->getModel()->getJointPosition(q);
//     _start_model->setJointPosition(q);
//     _start_model->update();
//     _goal_model->setJointPosition(q);
//     _goal_model->update();
    
    Eigen::VectorXd qG;
    _start_model->getJointPosition(qG);
    qG(0) += 3.0;
    _goal_model->setJointPosition(qG);
    _goal_model->update();
        
    // Goal Solver
    ik_yaml_goal = YAML::Load(problem_description);

    ci_period = 0.1;
    ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), _goal_model);

    ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);
    _ci_goal = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx);
    _goal_solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(_ci_goal);
    _rsc = std::make_shared<RosServerClass>(_ci_goal);    
}

void FootStepPlanner::init_load_planner() 
{
    // Create the state Compount State Space containing the feet State Spaces and assign the relative bounds
    if (!_planner_config["state_space"]["ee_number"])
    {
        throw std::runtime_error("Mandatory 'ee_number' private parameter in 'planner_config' missing");
    }
    YAML_PARSE_OPTION(_planner_config["state_space"], ee_number, int, 2);
    
    if (!_planner_config["state_space"]["end_effector"])
    {
        throw std::runtime_error("Mandatory 'end_effector' private parameter in 'planner_config' missing");
    }
    YAML_PARSE_OPTION(_planner_config["state_space"], end_effector, std::vector<std::string>, {});
    
    if (!_planner_config["state_space"]["bounds_x"])
    {
        throw std::runtime_error("Mandatory 'bounds_x' private parameter in 'planner_config' missing");
    }
    YAML_PARSE_OPTION(_planner_config["state_space"], bounds_x, std::vector<double>, {});
    
    if (!_planner_config["state_space"]["bounds_y"])
    {
        throw std::runtime_error("Mandatory 'bounds_y' private parameter in 'planner_config' missing");
    }
    YAML_PARSE_OPTION(_planner_config["state_space"], bounds_y, std::vector<double>, {});
    
    _ee_number = ee_number;
    _ee_name = end_effector;

    Eigen::Affine3d T;
    _model->getPose(_ee_name[0], T);
    _z_wheel = T.translation().z();
    _EE_rot = T.linear();
    
    
    
    ompl::base::RealVectorBounds bounds(2);
    if (bounds_x.size() > 0 && bounds_y.size() > 0)
    {       
        bounds.setLow(0, bounds_x[0]);
        bounds.setLow(1, bounds_y[0]);
        bounds.setHigh(0, bounds_x[1]);
        bounds.setHigh(1, bounds_y[1]);
    }   
    
    _space = std::make_shared<ompl::base::CompoundStateSpace>();
    
    for (int i = 0; i < _ee_number; i++)
    {   
        if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
        {
            auto spaces = std::make_shared<ompl::base::RealVectorStateSpace>(2);
            spaces->setBounds(bounds);
            _space->addSubspace(spaces, 1.0);
        }
        else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
        {
            auto spaces = std::make_shared<ompl::base::SE2StateSpace>();
            spaces->setBounds(bounds);
            _space->addSubspace(spaces, 1.0);
        }       
    }
        
    // Create the Compound Control Space
    auto foot_selector = std::make_shared<ompl::control::DiscreteControlSpace>(_space, 1, _ee_number);
    foot_selector->setControlSamplerAllocator(getSampler);
    
    auto step_size = std::make_shared<ompl::control::RealVectorControlSpace>(_space, _space->getSubspace(0)->getDimension());
    
    // Extra ControlSpace for esa_mirror robot with fixed step_size
//     auto step_size = std::make_shared<ompl::control::DiscreteControlSpace>(_space, 1, 6);
    
    YAML_PARSE_OPTION(_planner_config["control_space"], step_size_max, double, 1.0)
    ompl::base::RealVectorBounds step_bounds(_space->getSubspace(0)->getDimension());
    step_bounds.setLow(-1*step_size_max);
    step_bounds.setHigh(step_size_max);
    
    step_size->setBounds(step_bounds);
    
    _cspace = std::make_shared<ompl::control::CompoundControlSpace>(_space);
    _cspace->addSubspace(foot_selector);
    _cspace->addSubspace(step_size);
    
    
    // Create an instance to the space information
    _space_info = std::make_shared<ompl::control::SpaceInformation>(_space, _cspace);  
    
    _space_info->setDirectedControlSamplerAllocator(getDirectedControlSampler);
    
    YAML_PARSE_OPTION(_planner_config["control_space"], min_control_duration, int, 1);
    YAML_PARSE_OPTION(_planner_config["control_space"], max_control_duration, int, 10);
    
    _space_info->setMinMaxControlDuration(min_control_duration, max_control_duration);
}

void FootStepPlanner::init_load_goal_service() 
{   
    _start_goal_srv = _nh.advertiseService("compute_goal", &FootStepPlanner::start_goal_service, this);
}


void FootStepPlanner::init_load_state_propagator()
{
    if (!_planner_config["propagator"])
        throw std::runtime_error("Mandatory private paramenter 'propagator' missing");
    
    if (!_planner_config["propagator"]["duration"])
        std::cout << "Duration not set, using default value 0.1" << std::endl;
    
    YAML_PARSE_OPTION(_planner_config["propagator"], duration, double, 0.1);
    YAML_PARSE_OPTION(_planner_config["propagator"], type, std::string, "");
    
    _propagator_type = type;
    
    if (type == "centauro")
    {
        _propagator = std::make_shared<Planning::Propagators::stepPropagator_centauro>(_space_info, _sw);
    }
    else if (type == "tripod")
    {
        _propagator = std::make_shared<Planning::Propagators::stepPropagator_tripod>(_space_info, _sw);
    }
    else if (type == "" && _sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
    {
        _propagator = std::make_shared<Planning::Propagators::stepPropagator_wheel>(_space_info, _sw);
    }
    else if (type == "" && _sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
    {
        _propagator = std::make_shared<Planning::Propagators::stepPropagator_biped>(_space_info, _sw);
    }  
      
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
    
//     _goal_generator = std::make_shared<GoalGenerator>(_ci, _vc_context);
    _NSPG = std::make_shared<XBot::Cartesian::Planning::NSPG>(_solver, _vc_context);
}

void FootStepPlanner::setStateValidityPredicate(StateValidityPredicate svp)
{
    auto ompl_svc = [svp, this](const ompl::base::State * state)
    {       
        if (!_space_info->satisfiesBounds(state))
        {
            return false;
        }
        
        std::vector<Eigen::VectorXd> ee(_ee_number);  
        Eigen::Vector3d x_com = {0, 0, 0};
        double sum_x = 0;
        double sum_y = 0;
        Eigen::Affine3d T;
        
        // Transform state in Eigen::VectorXd and fill the IK solver
        for (int i = 0; i < _ee_number; i++)
        {
            if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
            {
                _sw->getState(state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), ee[i]);
                T.translation() << ee[i](0), ee[i](1), _z_wheel;
                T.linear() = _EE_rot;
            }
            else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)   
            {
                _sw->getState(state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(i), ee[i]);
                T.translation() << ee[i](0), ee[i](1), _z_wheel;
                T.linear() = _EE_rot;
                T.rotate( Eigen::AngleAxis<double>( ee[i](2), Eigen::Vector3d(0,0,1) )); 
            }
            _solver->setDesiredPose(_ee_name[i], T);
        }
        
//         std::cout << "---------FOOTSTEP PLANNER----------" << std::endl;
//         Eigen::Affine3d T_m;
//         _solver->getCI()->getPoseReference("wheel_1", T_m);
//         std::cout << "wheel1_ref: \n" << T_m.matrix() << std::endl;
//         
//         _solver->getCI()->getPoseReference("wheel_2", T_m);
//         std::cout << "wheel2_ref: \n" << T_m.matrix() << std::endl;
//         
//         _solver->getCI()->getPoseReference("wheel_3", T_m);
//         std::cout << "wheel3_ref: \n" << T_m.matrix() << std::endl;
//         
//         _solver->getCI()->getPoseReference("wheel_4", T_m);
//         std::cout << "wheel4_ref: \n" << T_m.matrix() << std::endl;
        
        // Check whether one of the two feet is in collision with the environment
        for (int i = 0; i < _ee_number; i++)
        {
            // TODO set size as parameter from config
            if (_map->checkForCollision(ee[i], 0.2))
            {
                return false;
            }
        }
               
        _model->setJointPosition(_qhome);
        _model->update();           
        
        // Extract the home state as a Eigen::VectorXd
        XBot::JointNameMap jmap, jmap_home;
        _ci->getReferencePosture(jmap);
        _ci->getReferencePosture(jmap_home);
        
        _model->eigenToMap(_qhome, jmap_home);
       
        // Check if the start state has been already explored and pick the relative postural
        // at the beginning, the postural is equal to the home state 
        
        if (_propagator_type == "centauro")
            step_propagator = std::dynamic_pointer_cast<XBot::Cartesian::Planning::Propagators::stepPropagator_centauro>(_propagator);
        
        else if (_propagator_type == "tripod")
            step_propagator = std::dynamic_pointer_cast<XBot::Cartesian::Planning::Propagators::stepPropagator_tripod>(_propagator);
        
        else if (_propagator_type == "" && _sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
        {
            step_propagator = std::dynamic_pointer_cast<XBot::Cartesian::Planning::Propagators::stepPropagator_wheel>(_propagator);
        }
        else if (_propagator_type == "" && _sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
        {
            step_propagator = std::dynamic_pointer_cast<XBot::Cartesian::Planning::Propagators::stepPropagator_biped>(_propagator);
        }    
        // Transform the start state in a std::vector
        auto sstate = step_propagator->getStartState();
        std::vector<double> state_vect;
                
        for (int i = 0; i < _ee_number; i++)
        {
            Eigen::VectorXd vect;
            if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
                _sw->getState(sstate->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), vect);
            else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
                _sw->getState(sstate->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(i), vect);
            for (int j = 0; j < vect.size(); j++)
                state_vect.push_back(vect(j));
        }
        
        // Look for the start state inside the unordered_map
        std::unordered_map<std::vector<double>, posturalStruct, posturalHash>::iterator it;
        it = _postural_map.find(state_vect);
        
        if (it != _postural_map.end())
        {
            Eigen::VectorXd qpost(_solver->getModel()->getJointNum());
            qpost = it->second.postural; 
            _model->eigenToMap(qpost, jmap);
            _ci->setReferencePosture(jmap);
        }
        else // reset postural to home state
        {            
            _ci->setReferencePosture(jmap_home);
        }
        
        
        Eigen::VectorXd x;
        
        // solve
        auto tic = std::chrono::high_resolution_clock::now();
        if (_solver->solve())
        {
            _solver->getModel()->getJointPosition(x);
            _model->setJointPosition(x);
            _model->update();
            
            if (!_vc_context.vc_aggregate.check("distance"))
                return false;          
            
            if (_goalSamplerType == "NSPG")// && (!_vc_context.vc_aggregate.check("collisions") || !_vc_context.vc_aggregate.check("stability")))
            {
//                XBot::Cartesian::Planning::NSPG::Ptr goal_sampler;
                _goalSampler_counter ++;
//                goal_sampler = std::make_shared<XBot::Cartesian::Planning::NSPG>(_solver, _vc_context);
                
                if (_NSPG->sample(1.5))
                {
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<float> fsec = toc - tic;
                    std::cout << "solution found in " << fsec.count() << " seconds" << std::endl;
                    _logger->add("success", 1);
                    _logger->add("time", fsec.count());
                    _solver->getModel()->getJointPosition(x);
                }
                else
                {
                    auto toc = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<float> fsec = toc - tic;
                    std::cout << "solution not found" << std::endl;
                    _logger->add("success", 0);
                    _logger->add("time", fsec.count());
                    _counter++;
                    return false;
                }
            }
            
//             if (!svp(x) && _goalSamplerType == "goalSampler" && (!_vc_context.vc_aggregate.check("collisions") || !_vc_context.vc_aggregate.check("stability")))
//             {
//                 _goalSampler_counter ++;
//                 if (!_goal_generator->samplePostural(x, 5.0))
//                 {
//                     _counter++;
//                     return false;
//                 }
//             }
                      
            // Add the new postural to the std::map together with the start state
            Eigen::VectorXd diff = x - _qhome;
            double err = diff.norm();
            std::vector<double> result_vect;
            for (int i = 0; i < _ee_number; i++)
            {
                Eigen::VectorXd vect;
                if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
                    _sw->getState(state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), vect);
                else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
                _sw->getState(state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(i), vect);
                for (int j = 0; j < vect.size(); j++)
                    result_vect.push_back(vect(j));
            }
            posturalStruct post_struct;
            post_struct.postural = x;
            post_struct.parent = state_vect;
            _trajectory_map.insert(std::make_pair(result_vect, post_struct));
            
            if (err > 0.0)
            {      
                _postural_map.insert(std::make_pair(result_vect, post_struct));  
            }
            
            return true; 
        }
        else
        {
            // IK solution not found, discarding state
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
}


bool FootStepPlanner::start_goal_service ( cartesio_planning::CartesioGoal::Request& req, cartesio_planning::CartesioGoal::Response& res ) 
{
    Eigen::Affine3d T;
    std::vector<double> before, after;
    double err, tolerance = 1.0;
    for (int i = 0; i < _ee_name.size(); i++)
    {
        _goal_model->getPose(_ee_name[i], T);
        if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
        {
            before.resize(_ee_number*2);
            before[2*i] = T.translation().x() * T.translation().x(); 
            before[2*1 + 1] = T.translation().y() * T.translation().y();
        }
        else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
        {
            before.resize(_ee_number*3);
            auto rpy = T.linear().eulerAngles(0, 1, 2);
            before[3*i] = T.translation().x() * T.translation().x();
            before[3*1 + 1] = T.translation().y() * T.translation().y();
            before[3*i + 2] = rpy[2] * rpy[2];
        }
    }
    _goal_solver->solve();
    
    check_state_valid(_goal_model);
    std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();        
    _goal_viz->publishMarkers(ros::Time::now(), red_links);
    
    XBot::Cartesian::Planning::NSPG::Ptr goal_sampler;
    goal_sampler = std::make_shared<XBot::Cartesian::Planning::NSPG>(_solver, _vc_context);
    if (goal_sampler->sample(req.time))
    {
        double err;
        
        // Set start and goal states        
        ompl::base::ScopedState<> goal(_space), start(_space);
        if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
        {
            
            for (int i = 0; i < _ee_name.size(); i++)
            {
                _goal_model->getPose(_ee_name[i], T);
                goal[2*i] = T.translation().x();
                goal[2*i+1] = T.translation().y();
                _start_model->getPose(_ee_name[i], T); 
                start[2*i] = T.translation().x();
                start[2*i+1] = T.translation().y();
            }
        }
        else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
        {
            for (int i = 0; i < _ee_name.size(); i++)
            {
                _goal_model->getPose(_ee_name[i], T);
                goal[3*i] = T.translation().x();
                goal[3*i + 1] = T.translation().y();
                auto rpy = T.linear().eulerAngles(0,1,2);
                goal[3*i + 2] = rpy[2];
                _start_model->getPose(_ee_name[i], T);
                start[3*i] = T.translation().x();
                start[3*i + 1] = T.translation().y();
                rpy = T.linear().eulerAngles(0,1,2);
                start[3*i + 2] = rpy[2];
            }
        }
        
        for (int i = 0; i < _ee_name.size(); i++)
        {
            _goal_model->getPose(_ee_name[i], T);
            if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
            {
                after.resize(_ee_number*2);
                after[2*i] = T.translation().x() * T.translation().x(); 
                after[2*1 + 1] = T.translation().y() * T.translation().y();
            }
            else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
            {
                after.resize(_ee_number*3);
                auto rpy = T.linear().eulerAngles(0, 1, 2);
                after[3*i] = T.translation().x() * T.translation().x();
                after[3*1 + 1] = T.translation().y() * T.translation().y();
                after[3*i + 2] = rpy[2] * rpy[2];
            }
        }
        
        // Compute error between interactive marker position and computed wheel position 
        for (int i = 0; i < after.size(); i++)
        {
            err += (after[i] - before[i]) * (after[i] - before[i]);
        }
        
        //TODO to be fixed
        err = sqrt(err);
        
        if (err < tolerance)
        {
            res.status.val = cartesio_planning::CartesioPlannerGoalStatus::EXACT_SOLUTION;
            std::cout << "EXACT SOLUTION FOUND...setting start and goal states!" << std::endl;
        }
        else
        {
            res.status.val = cartesio_planning::CartesioPlannerGoalStatus::APPROXIMATE_SOLUTION;
            std::cout << "APPROXIMATE SOLUTION FOUND...setting start and goal states! (error = " << err << ")" << std::endl;
        }
        _pdef->setStartAndGoalStates(start, goal, 2.0);
        return true;
    }
    else
    {
        res.status.val = cartesio_planning::CartesioPlannerGoalStatus::TIMEOUT;
        std::cout << "TIMEOUT REACHED...run service again or change goal state!" << std::endl;
        return false;
    }   
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
    
    _map->convert();
    
    res.status.val = callPlanner(req.time, req.planner_type);   
    
    if (res.status.val)
    {
        _path = _pdef->getSolutionPath();
        _path->print(std::cout);
        std::cout << "Goal Sampler called " << _goalSampler_counter << " times" << std::endl;
        std::cout << "Discarded " << _counter << " states due to goal_sampler" << std::endl;
        std::cout << "Created a map with: " << _postural_map.size() << " entries" << std::endl;
        
        ompl::control::PlannerData data(_space_info);
        _planner->getPlannerData(data);
        std::cout << "Solution Tree has " << data.numVertices() << " vertices and " << data.numEdges() << " edges" << std::endl;
        
        Eigen::VectorXd q_temp;
        Eigen::Affine3d T;
        std::vector<Eigen::VectorXd> ee(_ee_number);
        
        trajectory_msgs::JointTrajectory trj;
       
        auto last_state = _path->as<ompl::geometric::PathGeometric>()->getState(_path->as<ompl::geometric::PathGeometric>()->getStateCount()-1);
        auto start_state = _path->as<ompl::geometric::PathGeometric>()->getState(0);
        std::vector<double> last_state_vect, start_state_vect;
        
        for (int i = 0; i < _ee_number; i++)
        {
            Eigen::VectorXd start_vect, last_vect;
            if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
            {
                _sw->getState(start_state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), start_vect);
                _sw->getState(last_state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i), last_vect);
            }
            else if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::SE2SPACE)
            {
                _sw->getState(start_state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(i), start_vect);
                _sw->getState(last_state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(i), last_vect);
            }
            for (int j = 0; j < start_vect.size(); j++)
            {
                last_state_vect.push_back(last_vect(j));
                start_state_vect.push_back(start_vect(j));
            }
        }
        
        std::unordered_map<std::vector<double>, posturalStruct, posturalHash>::iterator it;
        it = _trajectory_map.find(last_state_vect);
        
        while (it->second.parent != start_state_vect)
        {
            _q_vect.push_back(it->second.postural);
            _state_vect.push_back(it->first);
            
            it = _trajectory_map.find(it->second.parent);
        }
        
        it = _trajectory_map.find(start_state_vect);
        _q_vect.push_back(it->second.postural);
        _state_vect.push_back(it->first);
        
        std::cout << "final q_vect size is " << _q_vect.size() << std::endl;
        
        std::reverse(_q_vect.begin(), _q_vect.end());
        std::reverse(_state_vect.begin(), _state_vect.end());
        
        std::vector<Eigen::VectorXd> q_fail;
        for (auto i : _q_vect)
        {
            _model->setJointPosition(i);
            _model->update();
            
            if (!_vc_context.vc_aggregate.check("collisions"))
            {
                q_fail.push_back(i);
            }
        }

        std::cout << "_q_vect failed size: " << q_fail.size() << std::endl;
//         interpolate();
        
//         for (int i = 0; i < data.numVertices(); i++)
//         {
//             Eigen::VectorXd foot;
//             Eigen::VectorXd state(8);
//             for (int j = 0; j < _ee_number; j++)
//             {
//                 if (_sw->getStateSpaceType() == Planning::StateWrapper::StateSpaceType::REALVECTOR)
//                 {
//                     _sw->getState(data.getVertex(i).getState()->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(j), foot);
//                     state(2*j) = foot(0);
//                     state(2*j + 1) = foot(1);
//                 }
//             }
//         }
        
        auto t = ros::Duration(0.);
        
        std::cout << "Final solution has " << _q_vect.size() << " steps!" << std::endl;

        _logger.reset();
        _NSPG->_logger.reset();
        
        for(auto x : _q_vect)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(x.data(), x.data() + x.size());
            point.time_from_start = t;
            trj.points.push_back(point);
            t += ros::Duration(1.);
        }
        
        trj.joint_names.assign(_model->getEnabledJointNames().data(), _model->getEnabledJointNames().data() + _model->getEnabledJointNames().size());
        
        _trj_publisher.publish(trj);              
    }
}

void FootStepPlanner::interpolate()
{
    // First, re-orient wheels in order to move to next state
    double T = 0.;
    double Tmax = 1.0;
    double dt = 0.01;
    
    std::vector<double> dtheta, yaw(4);
    _model->setJointPosition(_qhome);
    _model->update();
    XBot::JointNameMap jmap;
    _model->getJointPosition(jmap);
    std::vector<double> wheel_pos = {jmap["j_wheel_1"], jmap["j_wheel_2"], jmap["j_wheel_3"], jmap["j_wheel_4"]};
    std::vector<Eigen::VectorXd> q_fail;    
    
    for (int i = 0; i < _q_vect.size()-1; i++)
    {   
        std::vector<bool> fix_rot = {false, false, false, false};
        std::vector<int> inv_rot = {0, 0, 0, 0};
        
        for (int j = 0; j < _state_vect[i].size(); j += 2)
        {
            double theta;
            if (_state_vect[i+1][j] == _state_vect[i][j] && _state_vect[i+1][j+1] > _state_vect[i][j+1])
                theta = boost::math::constants::pi<double>()/2;
            
            else if (_state_vect[i+1][j] == _state_vect[i][j] && _state_vect[i+1][j+1] < _state_vect[i][j+1])
                theta = -boost::math::constants::pi<double>()/2;
              
            else if (_state_vect[i+1][j] == _state_vect[i][j] && _state_vect[i+1][j+1] == _state_vect[i][j+1])
            {
                if (j == 0 || j == 6)
                    theta = -boost::math::constants::pi<double>()/2;
                else
                    theta = boost::math::constants::pi<double>()/2;
                fix_rot[j/2] = true;
            }
            
            else
                theta = std::atan2((_state_vect[i+1][j+1] - _state_vect[i][j+1]), (_state_vect[i+1][j] - _state_vect[i][j]));
            
            
            dtheta.push_back(theta);           
        }
        yaw[0] = -dtheta[0] - jmap["hip_yaw_1"] + jmap["VIRTUALJOINT_6"];
        yaw[1] = -dtheta[1] - jmap["hip_yaw_2"] + jmap["VIRTUALJOINT_6"];
        yaw[2] = -dtheta[2] - jmap["hip_yaw_3"] + jmap["VIRTUALJOINT_6"];
        yaw[3] = -dtheta[3] - jmap["hip_yaw_4"] + jmap["VIRTUALJOINT_6"];
        std::for_each(yaw.begin(), yaw.end(), [&inv_rot, &yaw, &i](double& m)
            {
                if (m < -2.3)
                {
                    std::vector<double>::iterator it = std::find(yaw.begin(), yaw.end(), m);
                    unsigned int index = it - yaw.begin();
                    std::cout << "ankle_yaw_" << index << ": ";
                    std::cout << "angle " << m;
                    m += boost::math::constants::pi<double>();
                    std::cout << " modified in " << m << std::endl;
                    std::cout << " at stage " << i*100 << std::endl;
                    inv_rot[index] = -1;
                }
                else if (m > 2.3)
                {
                    std::vector<double>::iterator it = std::find(yaw.begin(), yaw.end(), m);
                    unsigned int index = it - yaw.begin();
                    std::cout << "ankle_yaw_" << index << ": ";
                    std::cout << "angle " << m;
                    m -= boost::math::constants::pi<double>();
                    std::cout << " modified in " << m << std::endl;
                    std::cout << " at stage " << i*100 << std::endl;
                    inv_rot[index] = 1;
                }
            });
        T = 0.;
        double dangle1 = yaw[0] - jmap["ankle_yaw_1"];
        double dangle2 = yaw[1] - jmap["ankle_yaw_2"];
        double dangle3 = yaw[2] - jmap["ankle_yaw_3"];
        double dangle4 = yaw[3] - jmap["ankle_yaw_4"];
        while (T < Tmax)
        {            
            jmap["ankle_yaw_1"] += dangle1/(Tmax/dt);
            jmap["ankle_yaw_2"] += dangle2/(Tmax/dt);
            jmap["ankle_yaw_3"] += dangle3/(Tmax/dt);
            jmap["ankle_yaw_4"] += dangle4/(Tmax/dt);
                         
            Eigen::VectorXd tmp(_model->getJointNum());
            _model->mapToEigen(jmap, tmp);
           
            _q_traj.push_back(tmp);
            T += dt;
        }
        
        // Move the whole robot
        T = 0.;
        Eigen::VectorXd tmp(_model->getJointNum());
        wheel_pos = {jmap["j_wheel_1"], jmap["j_wheel_2"], jmap["j_wheel_3"], jmap["j_wheel_4"]};
        while (T < Tmax)
        {            
            for (int j = 0; j < _q_vect[i].size(); j++)
            {              
                double a0, a1, a3;
                
                a3 = _q_vect[i](j);
                a0 = (2*_q_vect[i](j) - 2*_q_vect[i+1](j))/Tmax/Tmax/Tmax;
                a1 = -a0*Tmax + _q_vect[i+1](j)/Tmax/Tmax - _q_vect[i](j)/Tmax/Tmax;
                
                double q = a0*T*T*T + a1*T*T + a3;
                tmp(j) = q;             
            }
            _model->eigenToMap(tmp, jmap);
            if (inv_rot[0] == 1)
                jmap["ankle_yaw_1"] = -dtheta[0] - jmap["hip_yaw_1"] + jmap["VIRTUALJOINT_6"] - boost::math::constants::pi<double>();
            else if (inv_rot[0] == -1)
                jmap["ankle_yaw_1"] = -dtheta[0] - jmap["hip_yaw_1"] + jmap["VIRTUALJOINT_6"] + boost::math::constants::pi<double>();
            else 
                jmap["ankle_yaw_1"] = -dtheta[0] - jmap["hip_yaw_1"] + jmap["VIRTUALJOINT_6"];
            
            if (inv_rot[1] == 1)
                jmap["ankle_yaw_2"] = -dtheta[1] - jmap["hip_yaw_2"] + jmap["VIRTUALJOINT_6"] - boost::math::constants::pi<double>();
            else if (inv_rot[1] == -1)
                jmap["ankle_yaw_2"] = -dtheta[1] - jmap["hip_yaw_2"] + jmap["VIRTUALJOINT_6"] + boost::math::constants::pi<double>();
            else 
                jmap["ankle_yaw_2"] = -dtheta[1] - jmap["hip_yaw_2"] + jmap["VIRTUALJOINT_6"];
            
            if (inv_rot[2] == 1)
                jmap["ankle_yaw_3"] = -dtheta[2] - jmap["hip_yaw_3"] + jmap["VIRTUALJOINT_6"] - boost::math::constants::pi<double>();
            else if (inv_rot[2] == -1)
                jmap["ankle_yaw_3"] = -dtheta[2] - jmap["hip_yaw_3"] + jmap["VIRTUALJOINT_6"] + boost::math::constants::pi<double>();
            else 
                jmap["ankle_yaw_3"] = -dtheta[2] - jmap["hip_yaw_3"] + jmap["VIRTUALJOINT_6"];
            
            if (inv_rot[3] == 1)
                jmap["ankle_yaw_4"] = -dtheta[3] - jmap["hip_yaw_4"] + jmap["VIRTUALJOINT_6"] - boost::math::constants::pi<double>();
            else if (inv_rot[3] == -1)
                jmap["ankle_yaw_4"] = -dtheta[3] - jmap["hip_yaw_4"] + jmap["VIRTUALJOINT_6"] + boost::math::constants::pi<double>();
            else 
                jmap["ankle_yaw_4"] = -dtheta[3] - jmap["hip_yaw_4"] + jmap["VIRTUALJOINT_6"];
            
            // Rotate wheels
            std::vector<double> rot;
            for (int j = 0; j < _state_vect[i].size(); j += 2)
            {                  
                double distance = sqrt((_state_vect[i+1][j+1] - _state_vect[i][j+1])*(_state_vect[i+1][j+1] - _state_vect[i][j+1]) + (_state_vect[i+1][j] - _state_vect[i][j])*(_state_vect[i+1][j] - _state_vect[i][j]));
                rot.push_back(distance/0.078);
            }   
            
            for (int j = 0; j < rot.size(); j++)
            {
                double a0, a1, a3;
                
                a3 = wheel_pos[j];
                a0 = (2*wheel_pos[j] - 2*rot[j])/Tmax/Tmax/Tmax;
                a1 = -a0*Tmax + rot[j]/Tmax/Tmax - wheel_pos[j]/Tmax/Tmax;
                
                std::string num = std::to_string(j+1);
                if (j % 2 == 0)
                    jmap["j_wheel_" + num] = 3*a0*T*T + 2*a1*T;
                else
                    jmap["j_wheel_" + num] = -(3*a0*T*T + 2*a1*T);
            }
            
            if (fix_rot[0] == true)
                jmap["j_wheel_1"] = 0;
            if (fix_rot[1] == true)
                jmap["j_wheel_2"] = 0;
            if (fix_rot[2] == true)
                jmap["j_wheel_3"] = 0;
            if (fix_rot[3] == true)
                jmap["j_wheel_4"] = 0;
            
            if (inv_rot[0] != 0)
                jmap["j_wheel_1"] *= -1;
            if (inv_rot[1] != 0)
                jmap["j_wheel_2"] *= -1;
            if (inv_rot[2] != 0)
                jmap["j_wheel_3"] *= -1;
            if (inv_rot[3] != 0)
                jmap["j_wheel_4"] *= -1;
                     
            rot.clear();

            _model->mapToEigen(jmap, tmp);
            
            _model->setJointPosition(tmp);
            _model->update();
            
            _q_traj.push_back(tmp);
            T += dt;
        } 
       
        wheel_pos[0] = jmap["j_wheel_1"]; 
        wheel_pos[1] = jmap["j_wheel_2"];
        wheel_pos[2] = jmap["j_wheel_3"];
        wheel_pos[3] = jmap["j_wheel_4"];

        dtheta.clear();
    }
    
    // Check for collisions during interpolation
    auto config = XBot::ConfigOptionsFromParamServer();
    std::string urdf;
    
    if (!_nhpr.getParam("urdf", urdf))
        std::runtime_error("Mandatory private parameter 'urdf' missing!");
    
    config.set_urdf(urdf);
    _model = XBot::ModelInterface::getModel(config);
        
    _vc_context_interp = Planning::ValidityCheckContext(_planner_config, _model, _nh);
    
    _vc_context_interp.planning_scene->startMonitor();
    _vc_context_interp.planning_scene->startMonitor();
    
    ros::spinOnce();
    
    std::cout << "collisions after urdf change: " << q_fail.size() << std::endl;  
    
    config = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(config);
    
}


void FootStepPlanner::init_trajectory_publisher() 
{
    _trj_publisher = _nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, true);
    _xbotcore_trj_publisher = _nh.advertise<trajectory_msgs::JointTrajectory>("xbotcore_joint_trajectory", 1, true);
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
    
    auto tic = std::chrono::high_resolution_clock::now();   
    _status = _planner->solve(time);
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> fsec = toc-tic;
    
    std::cout << "SOLVER TOOK " << fsec.count() << " SECONDS TO FIND A SOLUTION" << std::endl;

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


ompl::control::DirectedControlSamplerPtr FootStepPlanner::getDirectedControlSampler ( const ompl::control::SpaceInformation* space_info ) 
{
    return std::make_shared<ompl::control::SimpleDirectedControlSampler>(space_info, 20);
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
        std::runtime_error("Selected planner does not exist!");
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
    
}

void FootStepPlanner::init_xbotcore_publisher()
{
    _publish_srv = _nh.advertiseService("publish_trajectory", &FootStepPlanner::publish_trajectory_service, this);
    _image_srv = _nh.advertiseService("image_service", &FootStepPlanner::image_service, this);
}

bool FootStepPlanner::publish_trajectory_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    trajectory_msgs::JointTrajectory trj;
    auto t = ros::Duration(0.);
        
        for(auto x : _q_traj)
        {
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions.assign(x.data(), x.data() + x.size());
            point.time_from_start = t;
            trj.points.push_back(point);
            t += ros::Duration(0.01);
        }
        
    trj.joint_names.assign(_model->getEnabledJointNames().data(), _model->getEnabledJointNames().data() + _model->getEnabledJointNames().size());
    
    _xbotcore_trj_publisher.publish(trj); 
    _trj_publisher.publish(trj);
}

bool FootStepPlanner::image_service ( std_srvs::Empty::Request& req, std_srvs::Empty::Response& res )
{
    // Reset the home position
    _start_model->setJointPosition(_qhome);
    _start_model->update();
    
    check_state_valid(_start_model);
    std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();
        
    _start_viz->publishMarkers(ros::Time::now(), red_links);
    
    std::vector<Eigen::VectorXd> vect_vect;
    Eigen::VectorXd vect;
    Eigen::VectorXd velocityLim;
    XBot::JointNameMap velocityLim_map, jmap, velocity_map;
    
    int iter = 0;
    double dt = 0.01;
    
    _model->eigenToMap(_qhome, jmap);
    _ci->setReferencePosture(jmap);
    velocityLim_map = jmap;
    velocity_map = jmap;
    
    _start_model->getVelocityLimits(velocityLim);
    _start_model->eigenToMap(velocityLim, velocityLim_map);
    
    XBot::Cartesian::Planning::NSPG::Ptr goal_sampler;               
    goal_sampler = std::make_shared<XBot::Cartesian::Planning::NSPG>(_solver, _vc_context);
    /*
    for(int i = 1; i < 8; i++)
    {
        std::string str = std::to_string(i);   
        double random = (double) (std::rand() - RAND_MAX/2) / (RAND_MAX/2);
        velocity_map["j_arm1_" + str] = random * velocityLim_map["j_arm1_" + str];  
    }
    
    double random = (double) (std::rand() - RAND_MAX/2) / (RAND_MAX/2);
    
    velocity_map["torso_yaw"] = random * velocityLim_map["torso_yaw"];*/
    
    for(int i = 1; i < 4; i++)
    {
        std::string str = std::to_string(i);   
        double random = (double) (std::rand() - RAND_MAX/2) / (RAND_MAX/2);
        velocity_map["VIRTUALJOINT_" + str] = random * 100;  
    }
    
    while(!check_state_valid(_start_model) && iter < 10)
    {
//         for (int i = 1; i < 8; i++)
//         {
//             std::string str = std::to_string(i);
//             jmap["j_arm1_" + str] += velocity_map["j_arm1_" + str] * dt;
//         }
//         jmap["torso_yaw"] += 2 * velocity_map["torso_yaw"] * dt;
        
        for (int i = 1; i < 4; i++)
        {
            std::string str = std::to_string(i);
            jmap["VIRTUALJOINT_" + str] += velocity_map["VIRTUALJOINT_" + str] * dt;
        }
        
        _ci->setReferencePosture(jmap);
        _solver->solve();
        _model->getJointPosition(vect);
        _start_model->setJointPosition(vect);
        _start_model->update();
        
        iter ++;
        
//         _start_model->mapToEigen(jmap, vect);
//         _start_model->setJointPosition(vect);
//         _start_model->update();
        
        vect_vect.push_back(vect);
    }
    
    std::cout << "Final vect_vect size: " << vect_vect.size() << std::endl;
    int i = 0;
    int counter = 0;
    
    while (counter < 1)
    {
        _start_model->setJointPosition(vect_vect[i]);
        _start_model->update();
        
        check_state_valid(_start_model);
        std::vector<std::string> red_links = _vc_context.planning_scene->getCollidingLinks();
        
        _start_viz->publishMarkers(ros::Time::now(), red_links);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        i++;
        
        if (i == vect_vect.size())
        {
            i = 0;
            counter++;
        }
    }
    
    return true;
}











