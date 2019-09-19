#include "cartesio_ompl_planner.h"
#include <type_traits>

using namespace XBot::Cartesian::Planning;

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max):
    _bounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR, _size);

    // create euclidean state space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // set bounds to state space
    setBounds(bounds_min, bounds_max);

    _space = _ambient_space;

    // create space information
    _space_info = std::make_shared<ompl::base::SpaceInformation>(_space);

    setUpProblemDefinition();

}

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
            const Eigen::VectorXd& bounds_max,
            ompl::base::ConstraintPtr constraint):
    _bounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::CONSTRAINED, _size);

    // create euclidean state space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // set bounds to state space
    setBounds(bounds_min, bounds_max);

    _space = std::make_shared<ompl::base::AtlasStateSpace>(_ambient_space, constraint);

    _space_info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(_space);

    setUpProblemDefinition();
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

    _ambient_space->setBounds(_bounds);

}

void OmplPlanner::setUpProblemDefinition()
{
     auto vss_alloc = [](const ompl::base::SpaceInformation * si)
     {
         auto vss = std::make_shared<ompl::base::UniformValidStateSampler>(si);
         vss->setNrAttempts(10000);
         return vss;
     };

     _space_info->setValidStateSamplerAllocator(vss_alloc);

    // create problem definition
    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);

    // set optimization objective (todo: provide choice to user)
    _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_space_info));
}


std::vector<Eigen::VectorXd> OmplPlanner::getSolutionPath() const
{
    auto * geom_path = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    std::vector<Eigen::VectorXd> path(geom_path->getStateCount());
    for(int i = 0; i < geom_path->getStateCount(); i++)
    {
        _sw->getState(geom_path->getState(i), path[i]);
    }

    return path;

}

ompl::base::SpaceInformationPtr OmplPlanner::getSpaceInfo() const { return _space_info; }

StateWrapper OmplPlanner::getStateWrapper() const { return *_sw; }



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

    auto atlas_ss = std::dynamic_pointer_cast<ompl::base::AtlasStateSpace>(_space);
    if(atlas_ss)
    {
        atlas_ss->anchorChart(ompl_start.get());
        atlas_ss->anchorChart(ompl_goal.get());
    }

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


bool OmplPlanner::solve(const double timeout, const std::string& planner_type)
{

    try
    {
        _planner = plannerFactory(planner_type);
    }
    catch(std::exception& e)
    {
        std::cout<<e.what()<<std::endl;
    }


    if(_planner)
    {
        _planner->setProblemDefinition(_pdef);
        _planner->setup();


        print();

        _solved = _planner->ompl::base::Planner::solve(timeout);

        if(_solved)
        {
            auto * geom_path = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

            geom_path->interpolate();

            if(!geom_path->check())
                return false;
        }

        return _solved;
    }
    return false;
}


void OmplPlanner::print(std::ostream &out)
{
    _space_info->printSettings(out);
    _pdef->print(out);
    _planner->printProperties(out);
}

ompl::base::PlannerStatus OmplPlanner::getPlannerStatus() const
{
    return _solved;
}



std::shared_ptr<ompl::base::Planner> OmplPlanner::plannerFactory(const std::string &planner_type)
{
    if(planner_type == "BiTRRT")
        return std::make_shared<ompl::geometric::BiTRRT>(_space_info);
    else if(planner_type == "InformedRRTstar")
        return std::make_shared<ompl::geometric::InformedRRTstar>(_space_info);
    else if(planner_type == "LazyLBTRRT")
        return std::make_shared<ompl::geometric::LazyLBTRRT>(_space_info);
    else if(planner_type == "LazyRRT")
        return std::make_shared<ompl::geometric::LazyRRT>(_space_info);
    else if(planner_type == "LBTRRT")
        return std::make_shared<ompl::geometric::LBTRRT>(_space_info);
    else if(planner_type == "pRRT")
        return std::make_shared<ompl::geometric::pRRT>(_space_info);
    else if(planner_type == "RRT")
        return std::make_shared<ompl::geometric::RRT>(_space_info);
    else if(planner_type == "RRTConnect")
        return std::make_shared<ompl::geometric::RRTConnect>(_space_info);
    else if(planner_type == "RRTsharp")
        return std::make_shared<ompl::geometric::RRTsharp>(_space_info);
    else if(planner_type == "RRTstar")
    {
        auto RRT_planner = std::make_shared<ompl::geometric::RRTstar>(_space_info);
        RRT_planner->setGoalBias(0.01);
        RRT_planner->setRange(1000);
        RRT_planner->setKNearest(true);
        return RRT_planner;
    }
    else if(planner_type == "RRTXstatic")
        return std::make_shared<ompl::geometric::RRTXstatic>(_space_info);
    else if(planner_type == "SORRTstar")
        return std::make_shared<ompl::geometric::SORRTstar>(_space_info);
    else if(planner_type == "TRRT")
        return std::make_shared<ompl::geometric::TRRT>(_space_info);
    //    else if(planner_type == "VFRRT")
    //        return std::make_shared<ompl::geometric::VFRRT>(space_info); ///TODO: implement options in factory
    else
        throw std::runtime_error("Planner type not valid!");
}
