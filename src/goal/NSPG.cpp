#include <cartesio_planning/goal/NSPG.h>

using namespace XBot::Cartesian::Planning;

static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context),
    _fb_step_size(1)
    {
        auto a = std::chrono::system_clock::now();
        time_t b = std::chrono::system_clock::to_time_t(a);
        randGenerator.seed(b);
        
        ros::NodeHandle rviz_nh("~");
        _rviz = std::make_shared<RobotViz>(ik_solver->getModel(),
                                           "nspg",
                                           rviz_nh,
                                           Eigen::Vector4d(1.0, 1.0, 0.1, 1.0)
                                           );
        _rviz->setPrefix("planner/");
    }
    
void NSPG::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

PositionCartesianSolver::Ptr NSPG::getIKSolver () const 
{
    return _ik_solver;
}

bool NSPG::sample ( double timeout ) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    
    _ik_solver->getModel()->getJointPosition(x);

    
    float T = 0.0;
    double dt = 0.001;
    int iter = 0;

    _time = std::chrono::high_resolution_clock::now();

    bool solved = _ik_solver->solve();

    if(solved)
        _rviz->publishMarkers(ros::Time::now(), {});

    std::vector<std::string> failed_predicate;
    
    while(!solved || !_vc_context.vc_aggregate.checkAll(&failed_predicate))
    {
        for (auto string : failed_predicate)
        {
            auto it = _fail_map.find(string);
            if (it != _fail_map.end())
            {
                it->second += 1;
            }
            else
            {
                _fail_map[string] = 1;
            }
        }
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            std::cout << "NSGP FAILS" << std::endl;
            for (auto pair : _fail_map)
            {
                std::cout << pair.first << ": " << pair.second << std::endl;
            }
            _fb_step_size = 1;
//            std::this_thread::sleep_for(std::chrono::seconds(2));
            return false;
        }
        
        // Acquire colliding chains
//        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        std::vector<XBot::ModelChain> colliding_chains {};
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 100 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);  
            _fb_step_size *= 10;
        }
                
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;

        _ik_solver->getCI()->setReferencePosture(joint_map);
        solved = _ik_solver->solve();
        if (!solved)
        {
            std::cout << "[NSPG]: unable to solve" << std::endl;
            auto toc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> fsec = toc-_time;
            T += fsec.count();
            _time = toc;
            continue;
        }

        _rviz->publishMarkers(ros::Time::now(), {});

        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-_time;
        T += fsec.count();

        _time = toc;
    }
    std::cout << "timeout" <<std::endl;
    std::cout << "NSGP SUCCESS" << std::endl;
    _fb_step_size = 1;
    for (auto pair : _fail_map)
    {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }
//    std::this_thread::sleep_for(std::chrono::seconds(2));
    return true;
}

double NSPG::generateRandom() 
{
    return randDistribution(randGenerator);
}

XBot::JointNameMap NSPG::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains) 
{
    XBot::JointNameMap random_map, chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;

    _ik_solver->getModel()->getVelocityLimits(velocity_lim);

    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);

    random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom() * _fb_step_size));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom() * _fb_step_size));
    random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom() * _fb_step_size));
    random_map.insert(std::make_pair("VIRTUALJOINT_4", generateRandom() * _fb_step_size));
    random_map.insert(std::make_pair("VIRTUALJOINT_5", generateRandom() * _fb_step_size));
    random_map.insert(std::make_pair("VIRTUALJOINT_6", generateRandom() * _fb_step_size));

    if (!_vc_context.vc_aggregate.check("collisions"))
    {
        for (auto i : colliding_chains)
        {
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
        }
    }

    return random_map;
}

XBot::ModelInterface::Ptr NSPG::getModel() const
{
    return _ik_solver->getModel();
}


