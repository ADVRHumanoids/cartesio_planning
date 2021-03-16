#include <ik/position_ik_solver.h>
#include <chrono>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <random>

#include <XBotInterface/ModelInterface.h>
#include "validity_checker/validity_checker_context.h"
#include "utils/robot_viz.h"
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <matlogger2/matlogger2.h>

namespace XBot { namespace Cartesian { namespace Planning {
    
    class NSPG {
           
    public: 
        
        typedef std::shared_ptr<NSPG> Ptr;
        
        NSPG(PositionCartesianSolver::Ptr ik_solver,
             Planning::ValidityCheckContext vc_context);
               
        void setIKSolver(PositionCartesianSolver::Ptr new_ik_solver);
        
        /**
         * @brief get the PositionCartesianSolver used by the NSPG
         * @return the PositionCartesianSolver::Ptr to be returned
         */        
        PositionCartesianSolver::Ptr getIKSolver() const;
        
        /**
         * @brief samples a new feasible state dependiong on the validity check function in vc_context
         * @arg timeout is the maximum allowed time given to the NSPG to find a solution
         * @return true if a solution is found in the given time
         */        
        
        ModelInterface::Ptr getModel() const;
        bool sample(double timeout);
        
        double generateRandom();
        XBot::MatLogger2::Ptr _logger;
        
    private:        
        XBot::JointNameMap generateRandomVelocities(std::vector<XBot::ModelChain> colliding_chains);
        
        PositionCartesianSolver::Ptr _ik_solver;
        
        Planning::ValidityCheckContext _vc_context;
        
        std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;        
        
    };
}}}
