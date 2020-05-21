#include <ik/position_ik_solver.h>
#include <chrono>
#include <functional>
#include <stdio.h>
#include <stdlib.h>

#include "validity_checker/validity_checker_context.h"

namespace XBot { namespace Cartesian { namespace Planning {
    
    class GoalSampler2 {
           
    public: 
        
        typedef std::shared_ptr<GoalSampler2> Ptr;

        GoalSampler2(PositionCartesianSolver::Ptr ik_solver,
                     Planning::ValidityCheckContext vc_context);
               
        void setIKSolver(PositionCartesianSolver::Ptr new_ik_solver);
        
        void getIKSolver(PositionCartesianSolver::Ptr &ik_solver) const;
        
        bool sample(double timeout);
        
    private: 
        double generateRandom();
        
        PositionCartesianSolver::Ptr _ik_solver;
        
        Planning::ValidityCheckContext _vc_context;
    };
}}}