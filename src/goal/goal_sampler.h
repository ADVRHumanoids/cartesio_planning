#ifndef GOAL_SAMPLER_H
#define GOAL_SAMPLER_H

#include <ompl/base/goals/GoalSampleableRegion.h>

#include "ik/position_ik_solver.h"
#include "state_wrapper.h"
#include <functional>

namespace XBot { namespace Cartesian { namespace Planning {
class GoalSamplerBase
{
public:
    typedef std::shared_ptr<GoalSamplerBase> Ptr;

    GoalSamplerBase(PositionCartesianSolver::Ptr ik_solver);

    /**
     * @brief setValidityCheker to set a function which expected result is true
     * @param validity_check
     */
    void setValidityCheker(const std::function<bool()> &validity_check);

    double distanceGoal(const Eigen::VectorXd& q) const;

    bool sampleGoal(Eigen::VectorXd& q, const unsigned int time_out_sec) const;

protected:
    std::function<bool()> _validity_check;
    PositionCartesianSolver::Ptr _ik_solver;
    Eigen::VectorXd _qmin, _qmax;

    Eigen::VectorXd generateRandomSeed() const;
};


class GoalSampler : public ompl::base::GoalSampleableRegion, GoalSamplerBase
{

public:

    typedef std::shared_ptr<GoalSampler> Ptr;

    GoalSampler(ompl::base::SpaceInformationPtr space_info,
                PositionCartesianSolver::Ptr ik_solver,
                StateWrapper state_wrapper);



public: // GoalRegion interface

    double distanceGoal(const ompl::base::State * st) const override;


public: // GoalSampleableRegion interface

    void sampleGoal(ompl::base::State * st) const override;
    unsigned int maxSampleCount() const override;

private:
    StateWrapper _state_wrapper;


};

} } }





#endif // GOAL_SAMPLER_H
