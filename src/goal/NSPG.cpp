#include <cartesio_planning/goal/NSPG.h>

using namespace XBot::Cartesian::Planning;

static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context)
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
    
    while(!solved || !_vc_context.vc_aggregate.checkAll())
    {
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            return false;
        }
        
        // Acquire colliding chains
//        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        std::vector<XBot::ModelChain> colliding_chains {};
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 50 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);  
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
    
    return true;
}

double NSPG::generateRandom() 
{
    return randDistribution(randGenerator);
}

XBot::JointNameMap NSPG::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains) 
{
    XBot::JointNameMap random_map, chain_map;

    random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom() * 50));
    random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom() * 100));
    random_map.insert(std::make_pair("VIRTUALJOINT_6", generateRandom() * 10));

    random_map.insert(std::make_pair("J1_E", generateRandom() * 50));
    random_map.insert(std::make_pair("J2_E", generateRandom() * 50));
    random_map.insert(std::make_pair("J3_E", generateRandom() * 50));
    random_map.insert(std::make_pair("J4_E", generateRandom() * 50));
    random_map.insert(std::make_pair("J5_E", generateRandom() * 50));
    random_map.insert(std::make_pair("J6_E", generateRandom() * 50));

    return random_map;
}

XBot::ModelInterface::Ptr NSPG::getModel() const
{
    return _ik_solver->getModel();
}


