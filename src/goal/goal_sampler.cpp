#include "goal_sampler.h"
#include <chrono>
#include <limits>

namespace XBot { namespace Cartesian { namespace Planning {

GoalSampler::GoalSampler(ompl::base::SpaceInformationPtr space_info,
                         PositionCartesianSolver::Ptr ik_solver,
                         StateWrapper state_wrapper):
    ompl::base::GoalSampleableRegion(space_info), GoalSamplerBase(ik_solver, ValidityCheckContext()),
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

    const_cast<GoalSampler*>(this)->GoalSamplerBase::sampleGoal(q, std::numeric_limits<int>::max());
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

GoalSamplerBase::GoalSamplerBase(PositionCartesianSolver::Ptr ik_solver, ValidityCheckContext vc_ctx):
    _ik_solver(ik_solver),
    _validity_check(vc_ctx)
{
    ik_solver->getModel()->getJointLimits(_qmin, _qmax);
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

    return error.squaredNorm();
}

bool GoalSamplerBase::sampleGoal(Eigen::VectorXd &q, double time_out_sec)
{
    // obtain model
    auto model = _ik_solver->getModel();

    bool goal_found = false;

    auto tstart = std::chrono::high_resolution_clock::now();

    Eigen::VectorXd qrand;
    model->getJointPosition(qrand);

    while(!goal_found)
    {

        // query ik
        goal_found = _ik_solver->solve();

        if(!goal_found)
        {
            Eigen::VectorXd err;
            _ik_solver->getError(err);
            printf("ik failed: error = %f > %f \n", err.lpNorm<Eigen::Infinity>(), _ik_solver->getErrorThreshold());
        }

        // if goal found, check state validity
        if(goal_found && !_validity_check.checkAll())
        {
            printf("found invalid goal state with colliding links: ");
            auto cl = _validity_check.planning_scene->getCollidingLinks();
            std::copy(cl.begin(), cl.end(), std::ostream_iterator<std::string>(std::cout, ", "));
            std::cout << std::endl;

            goal_found = false;
        }

        if(_cb)
        {
            _cb();
        }

        // update output
        model->getJointPosition(q);

        if(goal_found)
        {
            return true;
        }

        // generate random configuration
        auto qrand = generateRandomSeed();
        model->setJointPosition(qrand);
        model->update();

        if(_cb)
        {
            _cb();
        }

        // check timeout
        auto now = std::chrono::high_resolution_clock::now();

        std::chrono::duration<float> elapsed = now - tstart;

        if(elapsed.count() >= time_out_sec)
        {
            return false;
        }

    }

    return true;
}

bool GoalSamplerBase::sampleGoalPostural(Eigen::VectorXd &q, const unsigned int time_out_sec)
{
    auto model = _ik_solver->getModel();
    
    bool goal_found = false;
    
    auto tstart = std::chrono::high_resolution_clock::now();

    XBot::JointNameMap jmap;
    _ik_solver->getCI()->getReferencePosture(jmap);
    
    double neck_velodyne = jmap["neck_velodyne"];
    while (!goal_found)
    {
        
        // generate random configuration
        auto qrand = generateRandomSeed();
        
        // set it as postural;
        model->eigenToMap(qrand, jmap);
        jmap["neck_velodyne"] = neck_velodyne;
        _ik_solver->getCI()->setReferencePosture(jmap);
        
        // query ik
        goal_found = _ik_solver->solve();

        // if goal found, check state validity
        goal_found = goal_found && _validity_check.checkAll();

        // update output
        model->getJointPosition(q);

        if(goal_found)
        {
            return true;
        }

        // check timeout
        auto now = std::chrono::high_resolution_clock::now();

        std::chrono::duration<float> elapsed = now - tstart;

        if(elapsed.count() >= time_out_sec)
        {
            return false;
        }
    }
    return goal_found;
}

PositionCartesianSolver::Ptr GoalSamplerBase::getIkSolver()
{
    return _ik_solver;
}

void GoalSamplerBase::setIterationCallback(std::function<void ()> cb)
{
    _cb = cb;
}

void GoalSamplerBase::setJointLimits(Eigen::VectorXd qmin, Eigen::VectorXd qmax)
{
    if((qmax.array() < qmin.array()).any())
    {
        throw std::invalid_argument("invalid bounds");
    }

    _qmin = qmin;
    _qmax = qmax;
}

Eigen::VectorXd GoalSamplerBase::generateRandomSeed()
{
    // obtain model
    auto model = _ik_solver->getModel();

    // generate random configuration
    Eigen::VectorXd qrand, qcurr;

    model->getJointPosition(qcurr);

    qrand.setRandom(model->getJointNum()); // uniform in -1 < x < 1
    qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1
    qrand = _qmin + qrand.cwiseProduct(_qmax - _qmin); // uniform in qmin < x < qmax

//    if(model->isFloatingBase())
//    {
//        qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
//        qrand.head<3>() += qcurr.head<3>();
//        qrand.head<6>().tail<3>() *= M_PI;
//    }

//    qrand.head<6>().setZero();


    return qrand;
}





} } }
