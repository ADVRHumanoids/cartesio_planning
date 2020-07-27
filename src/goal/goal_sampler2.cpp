#include "goal_sampler2.h"

using namespace XBot::Cartesian::Planning;

GoalSampler2::GoalSampler2 ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context ):
    _ik_solver(ik_solver),
    _vc_context(vc_context)
    {}
    
void GoalSampler2::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

void GoalSampler2::getIKSolver ( PositionCartesianSolver::Ptr& ik_solver ) const 
{
    ik_solver = _ik_solver;
}

bool GoalSampler2::sample ( double timeout ) 
{
    // BE SURE THAT _ik_solver AND _vc_context HAS THE SAME MODEL
    Eigen::VectorXd x, dqlimits;
    XBot::JointNameMap chain_map, joint_map, velocity_map, random_map;
    
    // Start initializing joint_map
    _ik_solver->getCI()->getReferencePosture(joint_map);
    
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    
    _ik_solver->getModel()->getJointPosition(x);
    
    // Fill velocity_map with the velocity limits
    _ik_solver->getModel()->eigenToMap(x, velocity_map);
    _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
    
    float T = 0.0;
    double dt = 0.01;
    int iter = 0;
    
    while(!_vc_context.vc_aggregate.checkAll())
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 10 == 0)
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

double GoalSampler2::generateRandom() 
{
    return (double) (std::rand() - RAND_MAX/2) / (RAND_MAX/2);
}

XBot::JointNameMap GoalSampler2::generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains) 
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
            if (i.getChainName() == "front_right_leg" || i.getChainName() == "front_left_leg" || i.getChainName() == "rear_right_leg" || i.getChainName() == "rear_left_leg")
            {
                random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
                random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
            }
            
            if (i.getChainName() == "right_arm" || i.getChainName() == "left_arm")
            {
//                 random_map.insert(std::make_pair("torso_yaw", 2 * generateRandom() * velocityLim_map["torso_yaw"]));  // UNCOMMENT FOR CENTAURO
                random_map.insert(std::make_pair("WaistYaw", 2 * generateRandom() * velocityLim_map["WaistYaw"]));   // UNCOMMENT THIS FOR COMANPLUS
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

