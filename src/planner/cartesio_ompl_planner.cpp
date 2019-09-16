#include "cartesio_ompl_planner.h"
#include <type_traits>

using namespace XBot::Cartesian::Planning;

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max):
    _bounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR,
                                         _size);

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

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
            const Eigen::VectorXd& bounds_max,
            ompl::base::ConstraintPtr constraint):
    _bounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::CONSTRAINED,
                                         _size);

    // create euclidean state space
    _space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // set bounds to state space
    setBounds(bounds_min, bounds_max);

    setConstraint(constraint);
}

void OmplPlanner::setConstraint(ompl::base::ConstraintPtr constraint)
{
    _css = std::make_shared<ompl::base::ProjectedStateSpace>(_space, constraint);

    _csi = std::make_shared<ompl::base::ConstrainedSpaceInformation>(_css);

    // create space information
    _space_info = std::make_shared<ompl::base::SpaceInformation>(_css);

    // create problem definition
    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_csi);

    // set optimization objective (todo: provide choice to user)
    _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_csi));

    // create planner (todo: provide choice to user)
    _planner = std::make_shared<ompl::geometric::RRTstar>(_csi);
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
        _sw->getState(geom_path->getState(i), path[i]);
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


void OmplPlanner::setStateValidityPredicate(StateValidityPredicate svp)
{
    auto sw = _sw;

    auto ompl_svc = [sw, svp](const ompl::base::State * state)
    {
        Eigen::VectorXd x;
        sw->getState(state, x);

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


    _sw->setState(ompl_start.get(), start);
    _sw->setState(ompl_goal.get(), goal);


    // set start and goal
    _pdef->setStartAndGoalStates(ompl_start, ompl_goal);

}

void OmplPlanner::setStartAndGoalStates(const Eigen::VectorXd & start, ompl::base::GoalPtr goal)
{
    if(start.size() != _size)
    {
        throw std::invalid_argument("Invalid start/goal size: "
                                    "start size is " + std::to_string(start.size()) +
                                    ", expected size is " + std::to_string(_size));
    }

    // create ompl start and goal variables
    ompl::base::ScopedState<> ompl_start(_space);
    _sw->setState(ompl_start.get(), start);


    // set start and goal
    _pdef->clearStartStates();
    _pdef->addStartState(ompl_start);
    _pdef->setGoal(goal);
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


