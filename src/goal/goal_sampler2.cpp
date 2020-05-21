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
    XBot::JointNameMap chain_map, joint_map, velocity_map;
    _ik_solver->getCI()->getReferencePosture(joint_map);
    
    _ik_solver->getModel()->getVelocityLimits(dqlimits);
    
    _ik_solver->getModel()->getJointPosition(x);
    
    float T = 0.0;
    double dt = 0.01;
    
    while(!_vc_context.vc_aggregate.checkAll())
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        _ik_solver->getModel()->eigenToMap(x, joint_map);
        
        std::cout << "JOINT MAP BEFORE: \n";
        for (auto i : joint_map)
            std::cout << i.first << "                " << i.second << std::endl;
      
        velocity_map = joint_map;
        _ik_solver->getModel()->eigenToMap(dqlimits, velocity_map);
        
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        std::cout << "Colliding chains: \n";
        for (auto i : colliding_chains)
        {          
            std::cout << i.getChainName() << std::endl;
            i.getJointPosition(chain_map);
            
            for (auto j : chain_map)
            {
                j.second = generateRandom() * velocity_map[j.first];
                joint_map[j.first] += j.second * dt;
            }
        }
        
        std::cout << "\n JOINT MAP AFTER: \n";
        for (auto i : joint_map)
            std::cout << i.first << "                " << i.second << std::endl;
        
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
    return (double) (std::rand() - RAND_MAX) / RAND_MAX;
}






