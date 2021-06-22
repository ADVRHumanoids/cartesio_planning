#include "NSPG.h"

using namespace XBot::Cartesian::Planning;

static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-1.0, 1.0);

NSPG::NSPG ( PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_context):
    _ik_solver(ik_solver),
    _vc_context(vc_context),
    _vel_check(false)
{
        auto a = std::chrono::system_clock::now();
        time_t b = std::chrono::system_clock::to_time_t(a);
        randGenerator.seed(b);
        
        _rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(_ik_solver->getModel());
        XBot::MatLogger2::Options opt;
        opt.default_buffer_size = 1e6;
        std::string environment = getenv("ROBOTOLOGY_ROOT");
         _logger = XBot::MatLogger2::MakeLogger(environment + "/external/cartesio_planning/log/checks_log", opt);
         _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
    }
    
void NSPG::setIKSolver ( PositionCartesianSolver::Ptr new_ik_solver )
{
    _ik_solver = new_ik_solver;
}

PositionCartesianSolver::Ptr NSPG::getIKSolver () const 
{
    return _ik_solver;
}

bool NSPG::sample ( double timeout, int max_counter)
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
    
    _ik_solver->solve();
    _rspub->publishTransforms(ros::Time::now(), "/NSPG");
    
    float T = 0.0;
    double dt = 0.005;
    int iter = 0;
    unsigned int counter = 0;
    
//    check1 = _vc_context.vc_aggregate.check("collisions");
    check1 = _vc_context.vc_aggregate.check("collisions");
    check2 = _vc_context.vc_aggregate.check("stability");
    bool check = check1 && check2;

    int n_fail_coll = 0;
    int n_fail_stab = 0;
    int n_tot_call = 0;
        
    while(!check || counter < max_counter)
    {
        auto tic = std::chrono::high_resolution_clock::now();
        
        // Acquire colliding chains
        auto colliding_chains = _vc_context.planning_scene->getCollidingChains();
        
        // Generate a random velocity vector for colliding chains' joints only every n iterations
        if (iter % 10 == 0)
        {
            _ik_solver->getModel()->eigenToMap(x, joint_map);
            random_map = generateRandomVelocities(colliding_chains);
            _vel_check = false;
        }
         
        // Update joint_map with integrated random velocities       
        for (auto i : random_map)
            joint_map[i.first] += i.second * dt;
        
        iter ++;

        _ik_solver->getCI()->setReferencePosture(joint_map);
        auto tic_ik = std::chrono::high_resolution_clock::now();
        bool solved = _ik_solver->solve();
        auto toc_ik = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec_ik = toc_ik - tic_ik;
         _logger->add("time_ik", fsec_ik.count());
        if (!solved)
        {
            auto toc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> fsec = toc-tic;
            T += fsec.count();
            if(T >= timeout)
            {
                std::cout << "timeout" <<std::endl;
                return false;
            }
            continue;
        }
        
        if (_vc_context.vc_aggregate.checkAll())
            counter += 1;
        else
            counter = 0;


        _rspub->publishTransforms(ros::Time::now(), "");
                        
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec = toc-tic;
        T += fsec.count();
        if(T >= timeout)
        {
            std::cout << "timeout" <<std::endl;
            return false;
        }

        auto tic_coll = std::chrono::high_resolution_clock::now();
        check1 = _vc_context.vc_aggregate.check("collisions");
        if (!check1)
            n_fail_coll += 1;
//        check1 = true;
        auto toc_coll = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec_coll = toc_coll - tic_coll;
         _logger->add("collisions", fsec_coll.count());
        auto tic_stab = std::chrono::high_resolution_clock::now();
        check2 = _vc_context.vc_aggregate.check("stability");
        if(!check2)
            n_fail_stab += 1;
        auto toc_stab = std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> fsec_stab = toc_stab - tic_stab;
         _logger->add("stability", fsec_stab.count());

        check = check1 && check2;

        if (!check1 || !check2)
            n_tot_call += 1;

//        if (!check1 && !_vel_check)
//        {
//            auto coll_chains = _vc_context.planning_scene->getCollidingChains();
//            generateVelocityCollisions(coll_chains, random_map);
//            _vel_check = true;
//        }
        
//         if (!check1)
//             std::cout << "collision check failed!" << std::endl;
//         if (!check2)
//             std::cout << "stability check failed!" << std::endl;
//        if (check1)
//        {
//            for (auto& i : random_map)
//                i.second = 0;
//        }
//        if (!check2 && !_vel_check)
//        {
//            random_map.insert(std::make_pair("VIRTUALJOINT_1", 50*generateRandom()));
//            random_map.insert(std::make_pair("VIRTUALJOINT_2", 50*generateRandom()));
//            random_map.insert(std::make_pair("VIRTUALJOINT_3", 50*generateRandom()));
//            _vel_check = true;
//        }
//        if (check2)
//        {
//            random_map.insert(std::make_pair("VIRTUALJOINT_1", 0));
//            random_map.insert(std::make_pair("VIRTUALJOINT_2", 0));
//            random_map.insert(std::make_pair("VIRTUALJOINT_3", 0));
//        }


    }
    
    _logger->add("tot_calls", n_tot_call);
    _logger->add("coll_fails", n_fail_coll);
    _logger->add("stab_fails", n_fail_stab);

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
    if (!check1) 
    {
        for (auto i:colliding_chains)
        {
//             // Here you can add extra joints to the kinematic chains in collision.
             if (i.getChainName() == "front_right_leg" || i.getChainName() == "front_left_leg" || i.getChainName() == "rear_right_leg" || i.getChainName() == "rear_left_leg")
             {
                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*100));
                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*100));
                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*100));
             }
            
            if (i.getChainName() == "right_arm" || i.getChainName() == "left_arm")
            {
                random_map.insert(std::make_pair("torso_yaw", 4 * generateRandom() * velocityLim_map["torso_yaw"]));  // UNCOMMENT FOR CENTAURO
//                random_map.insert(std::make_pair("WaistYaw", 4 * generateRandom() * velocityLim_map["WaistYaw"]));   // UNCOMMENT THIS FOR COMANPLUS
            }
//             
//             if (i.getChainName() == "arm_A" || i.getChainName() == "arm_B" || i.getChainName() == "arm_C")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//             }

            if (i.getChainName() == "right_leg" || i.getChainName() == "left_leg")
            {
                random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*10));
                random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*10));
                random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*10));
            }

            /*random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
            random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
            random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));*/                
            
            i.getJointPosition(chain_map);
                
            for (auto j : chain_map)
            {
//                std::cout << j.first << std::endl;
                j.second = generateRandom() * velocityLim_map[j.first];
                random_map.insert(std::make_pair(j.first, j.second));
            }
            
        }
        
//        random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*10));
//        random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*20));
//        random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*10));
    }
    
    // Add random velocities to the floating base when the centroidal statics check fails
    if (!check2)
    {
        random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*150));
        random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*100));
        random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
    }

//     random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//     random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//     random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
    
    return random_map;
}

XBot::ModelInterface::Ptr NSPG::getModel() const
{
    return _ik_solver->getModel();
}

void NSPG::generateVelocityCollisions(std::vector<XBot::ModelChain> colliding_chains, XBot::JointNameMap &random_map)
{
    XBot::JointNameMap chain_map, velocityLim_map;
    Eigen::VectorXd velocity_lim;

    _ik_solver->getModel()->getVelocityLimits(velocity_lim);

    _ik_solver->getCI()->getReferencePosture(velocityLim_map);
    _ik_solver->getModel()->eigenToMap(velocity_lim, velocityLim_map);

    // Add random velocities for colliding chains

    for (auto i:colliding_chains)
    {
//             // Here you can add extra joints to the kinematic chains in collision.
         if (i.getChainName() == "front_right_leg" || i.getChainName() == "front_left_leg" || i.getChainName() == "rear_right_leg" || i.getChainName() == "rear_left_leg")
         {
             random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
             random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
             random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
         }

        if (i.getChainName() == "right_arm" || i.getChainName() == "left_arm")
        {
//                random_map.insert(std::make_pair("torso_yaw", 2 * generateRandom() * velocityLim_map["torso_yaw"]));  // UNCOMMENT FOR CENTAURO
            random_map.insert(std::make_pair("WaistYaw", 2 * generateRandom() * velocityLim_map["WaistYaw"]));   // UNCOMMENT THIS FOR COMANPLUS
        }
//
//             if (i.getChainName() == "arm_A" || i.getChainName() == "arm_B" || i.getChainName() == "arm_C")
//             {
//                 random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
//                 random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));
//             }

        if (i.getChainName() == "right_leg" || i.getChainName() == "left_leg")
        {
            random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*10));
            random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*10));
            random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*10));
        }

        /*random_map.insert(std::make_pair("VIRTUALJOINT_1", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_2", generateRandom()*50));
        random_map.insert(std::make_pair("VIRTUALJOINT_3", generateRandom()*50));*/

        i.getJointPosition(chain_map);

        for (auto j : chain_map)
        {
            j.second = generateRandom() * velocityLim_map[j.first];
            random_map.insert(std::make_pair(j.first, j.second));
        }

    }
}



