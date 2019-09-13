#include "cartesio_ompl_planner.h"
#include <type_traits>

using namespace XBot::Cartesian::Planning;

template<typename T>
OMPLPlanner<T>::OMPLPlanner(const Eigen::VectorXd& bounds_min, const Eigen::VectorXd& bounds_max):
_bounds(bounds_min.size()),
_size(bounds_min.size()),
_solved(ompl::base::PlannerStatus::UNKNOWN)
{
    static_assert(std::is_base_of<ompl::base::Planner, T>::value, "type parameter of this class must derive from ompl::base::Planner");

    _space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    if(!setBounds(bounds_min, bounds_max))
        throw std::runtime_error("Bounds incorrect size!");

    _space_info = std::make_shared<ompl::base::SpaceInformation>(_space);
    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);



    _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_space_info));


    _planner = std::make_shared<T>(_space_info);

    _planner->setProblemDefinition(_pdef);

    _planner->setup();

}

template<typename T>
ompl::base::PathPtr OMPLPlanner<T>::getSolutionPath()
{
    return _pdef->getSolutionPath();
}

template<typename T>
bool OMPLPlanner<T>::setBounds(const Eigen::VectorXd& bounds_min, const Eigen::VectorXd& bounds_max)
{
    if(bounds_min.size() != _size || bounds_max.size() != _size)
        return false;

    Eigen::VectorXd::Map(_bounds.low.data(), _size) = bounds_min;
    Eigen::VectorXd::Map(_bounds.high.data(), _size) = bounds_max;

    _space->setBounds(_bounds);
    return true;
}

template<typename T>
void OMPLPlanner<T>::setStateValidityChecker(const ompl::base::StateValidityCheckerPtr &svc)
{
    _space_info->setStateValidityChecker(svc);
}

template<typename T>
void OMPLPlanner<T>::setStateValidityChecker(const ompl::base::StateValidityCheckerFn &svc)
{
    _space_info->setStateValidityChecker(svc);
}

template<typename T>
bool OMPLPlanner<T>::setStartAndGoalStates(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal)
{
    _pdef->setStartAndGoalStates(start, goal);

    return true;
}

template<typename T>
bool OMPLPlanner<T>::setState(ompl::base::ScopedState<> state, const Eigen::VectorXd& value)
{
    if(value.size() != _size)
        return false;

    Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;

    return true;
}

template<typename T>
bool OMPLPlanner<T>::solve(const unsigned int t)
{
    print();

    _solved = _planner->ompl::base::Planner::solve(t);


    if( _solved ==  ompl::base::PlannerStatus::UNKNOWN ||
        _solved ==  ompl::base::PlannerStatus::INVALID_START ||
        _solved ==  ompl::base::PlannerStatus::INVALID_GOAL ||
        _solved ==  ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE ||
        _solved ==  ompl::base::PlannerStatus::TIMEOUT ||
        _solved ==  ompl::base::PlannerStatus::CRASH ||
        _solved ==  ompl::base::PlannerStatus::ABORT)
        return false;

    return true;
}

template<typename T>
void OMPLPlanner<T>::print()
{
    _space_info->printSettings(std::cout);
    _pdef->print(std::cout);
    ///...
}

template class OMPLPlanner<ompl::geometric::RRTstar>;  // Explicit instantiation

