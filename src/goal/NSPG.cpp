#include "NSPG.h"

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
        
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ik_solver->getModel());
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
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getModel()->getJointPosition(joint_map);
    
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    
    _ik_solver->getModel()->getJointPosition(x);
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    float T = 0.0;
    double dt = 0.01;
    int iter = 0;
    unsigned int counter = 0;
    unsigned int max_counter = 25;
    
    while(!_vc_context.vc_aggregate.checkAll() || counter < max_counter)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 100 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);  
        }
         
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;

        _ik_solver->getCI()->setReferencePosture(joint_map);
        _ik_solver->solve();
        
        if (_vc_context.vc_aggregate.checkAll())
            counter += 1;
        else
            counter = 0;

        _rspub->publishTransforms(ros::Time::now(), "/NSPG");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            return false;
        }
    }
    
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
    
    // Add random velocities for colliding chains
    if (!_vc_context.vc_aggregate.check("collisions")) 
    {
        for (auto i:colliding_chains)
        {
//             // Here you can add extra joints to the kinematic chains in collision.
//             if (i.getChainName() == "front_right_leg" || i.getChainName() == "front_left_leg" || i.getChainName() == "rear_right_leg" || i.getChainName() == "rear_left_leg")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//             }
//             
//             if (i.getChainName() == "right_arm" || i.getChainName() == "left_arm")
//             {
//                 random_map.insert(std::make_pair("torso_yaw", 2 * generateRandom() * velocityLim_map["torso_yaw"]));  // UNCOMMENT FOR CENTAURO
//                 random_map.insert(std::make_pair("WaistYaw", 2 * generateRandom() * velocityLim_map["WaistYaw"]));   // UNCOMMENT THIS FOR COMANPLUS
//             }
//             
//             if (i.getChainName() == "arm_A" || i.getChainName() == "arm_B" || i.getChainName() == "arm_C")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//             }
            
            if (i.getChainName() == "head")
            {
                random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));                
            }
            
            i.getJointPosition(chain_map);
                
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
            
        }
    }
    
    // Add random velocities to the floating base when the convex hull check fails
     if (!_vc_context.vc_aggregate.check("stability"))
     {
         random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
         random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
         random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
     }

    
    return random_map;
}

XBot::ModelInterface::Ptr NSPG::getModel() const
{
    return _ik_solver->getModel();
}


