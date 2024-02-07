#ifndef GOAL_SAMPLER_H
#define GOAL_SAMPLER_H

#include <ompl/base/goals/GoalSampleableRegion.h>

#include <cartesio_planning/ik/position_ik_solver.h>
#include <cartesio_planning/utils/state_wrapper.h>
#include <functional>
#include <cartesio_planning/validity_checker/validity_checker_context.h>

namespace XBot { namespace Cartesian { namespace Planning {
class GoalSamplerBase
{
public:

    typedef std::shared_ptr<GoalSamplerBase> Ptr;

    typedef std::function<Eigen::VectorXd(const Eigen::VectorXd&)> RandomSeedFn;

    GoalSamplerBase(PositionCartesianSolver::Ptr ik_solver,
                    ValidityCheckContext vc_ctx);

    void setJointLimits(Eigen::VectorXd qmin, Eigen::VectorXd qmax);

    double distanceGoal(const Eigen::VectorXd& q) const;

    bool sampleGoal(Eigen::VectorXd& q,
                    double time_out_sec,
                    RandomSeedFn random_seed_generator = RandomSeedFn());

    bool sampleGoalPostural(Eigen::VectorXd& q, const unsigned int time_out_sec);
    
    PositionCartesianSolver::Ptr getIkSolver();

    void setIterationCallback(std::function<void(void)> cb);

    Eigen::VectorXd generateRandomSeed();

    Eigen::VectorXd generateRandomSeedNullspace(const Eigen::VectorXd& qgoal, double dq);

protected:

    ValidityCheckContext _validity_check;
    PositionCartesianSolver::Ptr _ik_solver;
    Eigen::VectorXd _qmin, _qmax;
    Eigen::MatrixXd _J;
    Eigen::JacobiSVD<Eigen::MatrixXd> _Jsvd;


    std::function<void(void)> _cb;
};


class GoalSampler : public ompl::base::GoalSampleableRegion,
                    public GoalSamplerBase
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
