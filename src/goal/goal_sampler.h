#ifndef GOAL_SAMPLER_H
#define GOAL_SAMPLER_H

#include <ompl/base/goals/GoalSampleableRegion.h>

#include "../ik/position_ik_solver.h"
#include "../state_wrapper.h"

namespace XBot { namespace Cartesian { namespace Planning {

class GoalSampler : public ompl::base::GoalSampleableRegion
{

public:

    GoalSampler(ompl::base::SpaceInformationPtr space_info,
                PositionCartesianSolver::Ptr ik_solver,
                StateWrapper state_wrapper);


public: // GoalRegion interface

    double distanceGoal(const ompl::base::State * st) const override;


public: // GoalSampleableRegion interface

    void sampleGoal(ompl::base::State * st) const override;
    unsigned int maxSampleCount() const override;

private:

    Eigen::VectorXd generateRandomSeed() const;

    PositionCartesianSolver::Ptr _ik_solver;
    StateWrapper _state_wrapper;
    Eigen::VectorXd _qmin, _qmax;
};

} } }





#endif // GOAL_SAMPLER_H
