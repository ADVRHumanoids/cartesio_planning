#include "cartesio_ompl_planner.h"
#include <type_traits>

using namespace XBot::Cartesian::Planning;

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max):
    _bounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN)
{
    // create euclidean state space
    _space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // set bounds to state space
    setBounds(bounds_min, bounds_max);

    // create space information
    _space_info = std::make_shared<ompl::base::SpaceInformation>(_space);

    // create problem definition
    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);

    // set optimization objective (todo: provide choice to user)
    _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_space_info));

    // create planner (todo: provide choice to user)
    _planner = std::make_shared<ompl::geometric::RRTstar>(_space_info);
    _planner->setProblemDefinition(_pdef);
    _planner->setup();

}


std::vector<Eigen::VectorXd> OmplPlanner::getSolutionPath() const
{
    const int path_count = 100; // todo: from user

    std::vector<Eigen::VectorXd> path(path_count);

    auto * geom_path = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    geom_path->interpolate(path_count);

    for(int i = 0; i < path_count; i++)
    {
        auto * state_i = geom_path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
        path[i] = Eigen::VectorXd::Map(state_i->values, _size);
    }

    return path;

}


void OmplPlanner::setBounds(const Eigen::VectorXd& bounds_min,
                            const Eigen::VectorXd& bounds_max)
{
    if(bounds_min.size() != _size || bounds_max.size() != _size)
    {
        throw std::invalid_argument("Invalid bound size: "
                                    "min size is " + std::to_string(bounds_min.size()) +
                                    ", max size is " + std::to_string(bounds_max.size()) +
                                    ", expected size is " + std::to_string(_size));
    }

    if((bounds_max.array() < bounds_min.array()).any())
    {
        throw std::invalid_argument("Invalid bound value: max < min");
    }

    Eigen::VectorXd::Map(_bounds.low.data(), _size) = bounds_min;
    Eigen::VectorXd::Map(_bounds.high.data(), _size) = bounds_max;

    _space->setBounds(_bounds);

}


void OmplPlanner::setStateValidityChecker(StateValidityPredicate svp)
{
    const int size = _size;

    auto ompl_svc = [size, svp](const ompl::base::State * state)
    {
        const auto * state_rn = state->as<ompl::base::RealVectorStateSpace::StateType>();

        Eigen::VectorXd x = Eigen::VectorXd::Map(state_rn->values, size);

        return svp(x);
    };

    _space_info->setStateValidityChecker(ompl_svc);
}


void OmplPlanner::setStartAndGoalStates(const Eigen::VectorXd& start,
                                        const Eigen::VectorXd& goal)
{
    if(start.size() != _size || goal.size() != _size)
    {
        throw std::invalid_argument("Invalid start/goal size: "
                                    "start size is " + std::to_string(start.size()) +
                                    ", goal size is " + std::to_string(goal.size()) +
                                    ", expected size is " + std::to_string(_size));
    }

    // create ompl start and goal variables
    ompl::base::ScopedState<> ompl_start(_space);
    ompl::base::ScopedState<> ompl_goal(_space);

    // cast to Rn
    auto * ompl_start_rn = ompl_start->as<ompl::base::RealVectorStateSpace::StateType>();
    auto * ompl_goal_rn = ompl_goal->as<ompl::base::RealVectorStateSpace::StateType>();

    // fill from input
    Eigen::VectorXd::Map(ompl_start_rn->values, _size) = start;
    Eigen::VectorXd::Map(ompl_goal_rn->values, _size) = goal;

    // set start and goal
    _pdef->setStartAndGoalStates(ompl_start, ompl_goal);

}


bool OmplPlanner::setState(ompl::base::ScopedState<> state, const Eigen::VectorXd& value)
{
    if(value.size() != _size)
        return false;

    Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;

    return true;
}


bool OmplPlanner::solve(double t)
{
    print();

    _solved = _planner->ompl::base::Planner::solve(t);

    return _solved;
}


void OmplPlanner::print()
{
    _space_info->printSettings(std::cout);
    _pdef->print(std::cout);
    ///...
}

ompl::base::PlannerStatus OmplPlanner::getPlannerStatus() const
{
    return _solved;
}


