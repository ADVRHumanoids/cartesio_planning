#include "goal_sampler.h"

namespace XBot { namespace Cartesian { namespace Planning {

GoalSampler::GoalSampler(ompl::base::SpaceInformationPtr space_info,
                         PositionCartesianSolver::Ptr ik_solver,
                         StateWrapper state_wrapper):
    ompl::base::GoalSampleableRegion(space_info),
    _ik_solver(ik_solver),
    _state_wrapper(state_wrapper)
{

}

double GoalSampler::distanceGoal(const ompl::base::State * st) const
{

}

void GoalSampler::sampleGoal(ompl::base::State * st) const
{
    // obtain model
    auto model = _ik_solver->getModel();

    bool goal_found = false;

    while(!goal_found)
    {
        // generate random configuration
        auto qrand = generateRandomSeed();

        // set it to the model
        model->setJointPosition(qrand);
        model->update();

        goal_found = _ik_solver->solve();

    }



}

unsigned int GoalSampler::maxSampleCount() const
{
}

Eigen::VectorXd GoalSampler::generateRandomSeed() const
{
    // obtain model
    auto model = _ik_solver->getModel();

    // generate random configuration
    Eigen::VectorXd qrand;
    qrand.setRandom(model->getJointNum()); // uniform in -1 < x < 1
    qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1

    qrand = _qmin + qrand.cwiseProduct(_qmax - _qmin); // uniform in qmin < x < qmax
    qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)

    return qrand;
}

} } }
