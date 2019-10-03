#include "goal_sampler.h"
#include <chrono>
#include <limits>

namespace XBot { namespace Cartesian { namespace Planning {

GoalSampler::GoalSampler(ompl::base::SpaceInformationPtr space_info,
                         PositionCartesianSolver::Ptr ik_solver,
                         StateWrapper state_wrapper):
    ompl::base::GoalSampleableRegion(space_info), GoalSamplerBase(ik_solver),
    _state_wrapper(state_wrapper)
{
    setThreshold(ik_solver->getErrorThreshold());
}

double GoalSampler::distanceGoal(const ompl::base::State * st) const
{       
    // set state into model
    Eigen::VectorXd q;
    _state_wrapper.getState(st, q);

    return GoalSamplerBase::distanceGoal(q);
}

void GoalSampler::sampleGoal(ompl::base::State * st) const
{
    Eigen::VectorXd q;
    GoalSamplerBase::sampleGoal(q, std::numeric_limits<int>::max());
    _state_wrapper.setState(st, q);

    if(!isSatisfied(st))
    {
        throw std::runtime_error("Something went wrong with random goal generation");
    }

}

unsigned int GoalSampler::maxSampleCount() const
{
    return std::numeric_limits<unsigned int>::max();
}

GoalSamplerBase::GoalSamplerBase(PositionCartesianSolver::Ptr ik_solver):
    _ik_solver(ik_solver)
{
    ik_solver->getModel()->getJointLimits(_qmin, _qmax);
}

void GoalSamplerBase::setValidityCheker(const std::function<bool ()> &validity_check)
{
    _validity_check = validity_check;
}

double GoalSamplerBase::distanceGoal(const Eigen::VectorXd &q) const
{
    // get model
    auto model = _ik_solver->getModel();

    // set state into model
    model->setJointPosition(q);
    model->update();

    Eigen::VectorXd error;
    _ik_solver->getError(error);

    return error.norm();
}

bool GoalSamplerBase::sampleGoal(Eigen::VectorXd &q, const unsigned int time_out_sec) const
{
    // obtain model
    auto model = _ik_solver->getModel();

    bool goal_found = false;

    while(!goal_found)
    {
        auto tic = std::chrono::high_resolution_clock::now();

        // generate random configuration
        auto qrand = generateRandomSeed();

        // set it to the model
        model->setJointPosition(qrand);
        model->update();

        goal_found = _ik_solver->solve();

        if(_validity_check)
            goal_found = goal_found && _validity_check();

        model->getJointPosition(q);

        auto toc = std::chrono::high_resolution_clock::now();

        std::chrono::duration<float> fsec = toc-tic;
        if(fsec.count() >= time_out_sec)
            return false;
    }

    return true;
}

Eigen::VectorXd GoalSamplerBase::generateRandomSeed() const
{
    // obtain model
    auto model = _ik_solver->getModel();

    // generate random configuration
    Eigen::VectorXd qrand;
    qrand.setRandom(model->getJointNum()); // uniform in -1 < x < 1
    qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1

    qrand = _qmin + qrand.cwiseProduct(_qmax - _qmin); // uniform in qmin < x < qmax

    if(model->isFloatingBase())
    {
        qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
        qrand.head<6>().tail<3>() *= M_PI;
    }

    return qrand;
}





} } }
