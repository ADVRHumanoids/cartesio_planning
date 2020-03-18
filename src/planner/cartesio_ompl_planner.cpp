#include <type_traits>



#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/VFRRT.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#define remove_cv_t remove_cv // this fixes a missing typedef in SPARStwo header
#include <ompl/geometric/planners/prm/SPARStwo.h>
#undef  remove_cv_t
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/objectives/MinimaxObjective.h>
#include <ompl/base/PlannerData.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>

#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include "cartesio_ompl_planner.h"
#include "utils/parse_yaml_utils.h"

#include <matlogger2/matlogger2.h>

using namespace XBot::Cartesian::Planning;

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max,
                         YAML::Node options):
    _sbounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN),
    _options(options)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR, _size);

    // create euclidean state space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // unconstrained case -> state space = ambient state space
    _space = _ambient_space;

    // set bounds to state space
    set_bounds(bounds_min, bounds_max);

    // create space information
    _space_info = std::make_shared<ompl::base::SpaceInformation>(_space);

    // setup problem definition
    setup_problem_definition();

}

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max,
                         double control_min,
                         double control_max,
                         YAML::Node options):
    _sbounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN),
    _options(options)
{
    _cbounds = std::make_shared<ompl::base::RealVectorBounds>(2);

    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR, _size);

    // create euclidean state space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // unconstrained case -> state space = ambient state space
    _space = _ambient_space;

    // set bounds to state space
    set_bounds(bounds_min, bounds_max);
    
    // create control space
    _cspace = std::make_shared<ompl::control::RealVectorControlSpace>(_space, 2);
    std::cout << "Create a control space" << std::endl;

    
    // set bounds to control space
    _cbounds->setLow(control_min);
    _cbounds->setHigh(control_max);
    _cspace->setBounds(*_cbounds);

    // create space information
    _cspace_info = std::make_shared<ompl::control::SpaceInformation>(_space, _cspace);
    std::cout << "Real Vector control space info created" << std::endl;


    // setup problem definition
    //setup_problem_definition();

}

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max,
                         ompl::base::ConstraintPtr constraint,
                         YAML::Node options):
    _sbounds(bounds_min.size()),
    _constraint(constraint),
    _solved(ompl::base::PlannerStatus::UNKNOWN),
    _size(bounds_min.size()),
    _options(options)
{
    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::CONSTRAINED, _size);

    // create euclidean state space as ambient space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // constrained state space
    _space = make_constrained_space();

    // set bounds to state space
    set_bounds(bounds_min, bounds_max);

    // create **constrained** space information
    _space_info = std::make_shared<ompl::base::ConstrainedSpaceInformation>(_space);
    // setup problem definition
    setup_problem_definition();
}

OmplPlanner::OmplPlanner(const Eigen::VectorXd& bounds_min,
                         const Eigen::VectorXd& bounds_max,
                         double control_min,
                         double control_max,
                         ompl::base::ConstraintPtr constraint,
                         YAML::Node options):
    _sbounds(bounds_min.size()),
    _size(bounds_min.size()),
    _solved(ompl::base::PlannerStatus::UNKNOWN),
    _options(options)
{
    _cbounds = std::make_shared<ompl::base::RealVectorBounds>(2);

    _sw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR, _size);

    // create euclidean state space
    _ambient_space = std::make_shared<ompl::base::RealVectorStateSpace>(_size);

    // unconstrained case -> state space = ambient state space
    _space = make_constrained_space();

    // set bounds to state space
    set_bounds(bounds_min, bounds_max);
    
    // create control space
    _cspace = std::make_shared<ompl::control::RealVectorControlSpace>(_space, 2);
    
    // set bounds to control space
    _cbounds->setLow(control_min);
    _cbounds->setHigh(control_max);
    _cspace->setBounds(*_cbounds);

    // create space information
    _cspace_info = std::make_shared<ompl::control::SpaceInformation>(_space, _cspace);

    // setup problem definition
    setup_problem_definition();

}

void OmplPlanner::set_bounds(const Eigen::VectorXd& bounds_min,
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

    Eigen::VectorXd::Map(_sbounds.low.data(), _size) = bounds_min;
    Eigen::VectorXd::Map(_sbounds.high.data(), _size) = bounds_max;

    _ambient_space->setBounds(_sbounds);

}

void OmplPlanner::setup_problem_definition()
{
    auto vss_alloc = [](const ompl::base::SpaceInformation * si)
    {
        auto vss = std::make_shared<ompl::base::UniformValidStateSampler>(si);
        vss->setNrAttempts(10000);
        return vss;
    };
    
    _space_info->setValidStateSamplerAllocator(vss_alloc);

    // create problem definition
    if (!_options["planner"])
    {
        _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);
        _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_space_info));
    }
    else
    {
        _pdef = std::make_shared<ompl::base::ProblemDefinition>(_cspace_info);
        _pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(_cspace_info));
    }
}


ompl::base::PlannerPtr OmplPlanner::make_RRTstar()
{

    auto planner = std::make_shared<ompl::geometric::RRTstar>(_space_info);

    if(!_options || !_options["RRTstar"])
    {
        std::cout << "No options detected" << std::endl;
        return planner;
    }

    auto opt = _options["RRTstar"];

    PLANNER_PARSE_OPTION(GoalBias, double);
    PLANNER_PARSE_OPTION(Range, int);
    PLANNER_PARSE_OPTION(KNearest, bool);

    return planner;
}

ompl::base::PlannerPtr OmplPlanner::make_RRTConnect()
{
    auto planner = std::make_shared<ompl::geometric::RRTConnect>(_space_info);

    if(!_options || !_options["RRTConnect"])
    {
        std::cout << "No options detected" << std::endl;
        return planner;
    }

    auto opt = _options["RRTConnect"];

    PLANNER_PARSE_OPTION(IntermediateStates, bool);
    PLANNER_PARSE_OPTION(Range, double);

    return planner;
}

ompl::base::StateSpacePtr OmplPlanner::make_constrained_space()
{

    std::string css_type = "Atlas";

    std::cout << _options << std::endl;

    if(_options && _options["state_space"] && _options["state_space"]["type"])
    {
        css_type = _options["state_space"]["type"].as<std::string>();
    }
    else
    {
        std::cout << "No state_space/type found, using default '" << css_type << "'" << std::endl;
    }

#define ADD_CSS_AND_IF(css_name) \
    valid_css.push_back(css_name); if(css_type == css_name)

    std::vector<std::string> valid_css;

    ADD_CSS_AND_IF("Atlas")
    {
        return make_atlas_space();
    }

    ADD_CSS_AND_IF("TangentBundle")
    {
        return make_tangent_bundle();
    }

    ADD_CSS_AND_IF("Projected")
    {
        return make_projected_space();
    }

    std::cout << "Valid constrained state spaces are \n";
    std::for_each(valid_css.begin(), valid_css.end(),
                  [](std::string p)
    {
        std::cout << " - " << p << "\n";
    });
    std::cout.flush();

    throw std::runtime_error("Planner type '" + css_type + "' not valid!");
}

ompl::base::StateSpacePtr OmplPlanner::make_atlas_space()
{    
    _on_set_start_goal = [this](const ompl::base::State* start,
            const ompl::base::State* goal)
    {
        auto atlas_ss = std::dynamic_pointer_cast<ompl::base::AtlasStateSpace>(_space);
        if(atlas_ss)
        {
            atlas_ss->anchorChart(start);
            atlas_ss->anchorChart(goal);
        }
        else
        {
            throw std::runtime_error("dynamic_pointer_cast to 'ompl::base::AtlasStateSpace' failed");
        }
    };

    auto atlas_ss =  std::make_shared<ompl::base::AtlasStateSpace>(_ambient_space, _constraint);
    if(_options && _options["state_space"])
    {
        YAML_PARSE_OPTION(_options["state_space"], exploration, double, 0.5);
        YAML_PARSE_OPTION(_options["state_space"], delta, double, 0.1);
	YAML_PARSE_OPTION(_options["state_space"], epsilon, double, 0.5);
        YAML_PARSE_OPTION(_options["state_space"], alpha, double, M_PI/6);
        YAML_PARSE_OPTION(_options["state_space"], rho, double, 0.5);
        YAML_PARSE_OPTION(_options["state_space"], lambda, double, 50.0);
	
	atlas_ss->setExploration(exploration);
	atlas_ss->setDelta(delta);
	atlas_ss->setEpsilon(epsilon);
	atlas_ss->setAlpha(alpha);
	atlas_ss->setRho(rho);
	atlas_ss->setLambda(lambda);
    }
    
    return atlas_ss;
}

ompl::base::StateSpacePtr OmplPlanner::make_tangent_bundle()
{
    make_atlas_space();

    return std::make_shared<ompl::base::TangentBundleStateSpace>(_ambient_space, _constraint);
}

ompl::base::StateSpacePtr OmplPlanner::make_projected_space()
{
    return std::make_shared<ompl::base::ProjectedStateSpace>(_ambient_space, _constraint);
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

ompl::base::SpaceInformationPtr OmplPlanner::getSpaceInfo() const
{
    return _space_info;
}

ompl::control::SpaceInformationPtr OmplPlanner::getCSpaceInfo() const 
{
    return _cspace_info;
}


void OmplPlanner::getBounds(Eigen::VectorXd & qmin, Eigen::VectorXd & qmax) const
{
    qmin = qmin.Map(_sbounds.low.data(), _sbounds.low.size());
    qmax = qmax.Map(_sbounds.high.data(), _sbounds.high.size());
}

StateWrapper OmplPlanner::getStateWrapper() const
{
    return *_sw;
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
    
    if (!_options["planner"])
        _space_info->setStateValidityChecker(ompl_svc);
    else
        _cspace_info->setStateValidityChecker(ompl_svc);
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
    
    // this resets problem definition
    if (!_options["planner"])
        setup_problem_definition();
    else    
        _pdef = std::make_shared<ompl::base::ProblemDefinition>(_cspace_info);
 
    // set start and goal
    _pdef->setStartAndGoalStates(ompl_start, ompl_goal);
    
    // trigger callback
    if(_on_set_start_goal)
    {
        _on_set_start_goal(ompl_start.get(), ompl_goal.get());
    }

}


void OmplPlanner::setStartAndGoalStates(const Eigen::VectorXd & start,
                                        std::shared_ptr<ompl::base::GoalSampleableRegion> goal)
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

    auto atlas_ss = std::dynamic_pointer_cast<ompl::base::AtlasStateSpace>(_space);
    if(atlas_ss)
    {
        atlas_ss->anchorChart(ompl_start.get());
        ompl::base::ScopedState<> ompl_goal(_space);

        for(int i = 0; i < 100; i++)
        {
            goal->sampleGoal(ompl_goal.get());
            atlas_ss->anchorChart(ompl_goal.get());
        }
    }
}

bool OmplPlanner::solve(const double timeout, const std::string& planner_type)
{
    _planner = make_planner(planner_type);

    if(_planner)
    {
        _planner->setProblemDefinition(_pdef);
        _planner->setup();

        print();

        _solved = _planner->ompl::base::Planner::solve(timeout);
        
        if(_solved)
        {
            auto * geom_path = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

//             geom_path->interpolate();

            if(!geom_path->check())
                return false;
        }
        
        return _solved;
    }
    return false;
}


void OmplPlanner::print(std::ostream &out)
{
    if (!_options["planner"])
        _space_info->printSettings(out);
    else
        _cspace_info->printSettings(out);
    _pdef->print(out);
    _planner->printProperties(out);
}

ompl::base::PlannerStatus OmplPlanner::getPlannerStatus() const
{
    return _solved;
}


ompl::base::PlannerPtr OmplPlanner::make_planner(const std::string &planner_type)
{

#define ADD_PLANNER_AND_IF(planner_name) \
    valid_planners.push_back(planner_name); if(planner_type == planner_name)

    std::vector<std::string> valid_planners;

    ADD_PLANNER_AND_IF("BiTRRT")
    {
        return std::make_shared<ompl::geometric::BiTRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("InformedRRTstar")
    {
        return std::make_shared<ompl::geometric::InformedRRTstar>(_space_info);
    }

    ADD_PLANNER_AND_IF("LazyLBTRRT")
    {
        return std::make_shared<ompl::geometric::LazyLBTRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("LazyRRT")
    {
        return std::make_shared<ompl::geometric::LazyRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("LBTRRT")
    {
        return std::make_shared<ompl::geometric::LBTRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("pRRT")
    {
        return std::make_shared<ompl::geometric::pRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("RRT")
    {
        return std::make_shared<ompl::geometric::RRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("RRTConnect")
    {
        return make_RRTConnect();      
    }

    ADD_PLANNER_AND_IF("RRTsharp")
    {
        return std::make_shared<ompl::geometric::RRTsharp>(_space_info);
    }

    ADD_PLANNER_AND_IF("RRTstar")
    {
        return make_RRTstar();
    }

    ADD_PLANNER_AND_IF("RRTXstatic")
    {
        return std::make_shared<ompl::geometric::RRTXstatic>(_space_info);
    }

    ADD_PLANNER_AND_IF("SORRTstar")
    {
        return std::make_shared<ompl::geometric::SORRTstar>(_space_info);
    }

    ADD_PLANNER_AND_IF("TRRT")
    {
        return std::make_shared<ompl::geometric::TRRT>(_space_info);
    }

    ADD_PLANNER_AND_IF("PRM")
    {
        return std::make_shared<ompl::geometric::PRM>(_space_info);
    }

    ADD_PLANNER_AND_IF("PRMstar")
    {
        return std::make_shared<ompl::geometric::PRMstar>(_space_info);
    }

    ADD_PLANNER_AND_IF("LazyPRMstar")
    {
        return std::make_shared<ompl::geometric::LazyPRMstar>(_space_info);
    }

    ADD_PLANNER_AND_IF("LazyPRM")
    {
        return std::make_shared<ompl::geometric::LazyPRM>(_space_info);
    }

    ADD_PLANNER_AND_IF("SPARS")
    {
        return std::make_shared<ompl::geometric::SPARS>(_space_info);
    }

    ADD_PLANNER_AND_IF("SPARStwo")
    {
        return std::make_shared<ompl::geometric::SPARStwo>(_space_info);
    }

    ADD_PLANNER_AND_IF("KPIECE1")
    {
        return std::make_shared<ompl::geometric::KPIECE1>(_space_info);
    }

    ADD_PLANNER_AND_IF("BKPIECE1")
    {
        return std::make_shared<ompl::geometric::BKPIECE1>(_space_info);
    }

    ADD_PLANNER_AND_IF("LBKPIECE1")
    {
        return std::make_shared<ompl::geometric::LBKPIECE1>(_space_info);
    }
    
    ADD_PLANNER_AND_IF("SyclopRRT")
    {
        auto decomp = std::make_shared<MyGridDecomposition>(32,_sbounds);
        return std::make_shared<ompl::control::SyclopRRT>(_cspace_info, decomp);
    }
    
    ADD_PLANNER_AND_IF("SyclopEST")
    {
        auto decomp = std::make_shared<MyGridDecomposition>(32,_sbounds);
        return std::make_shared<ompl::control::SyclopEST>(_cspace_info, decomp);
    }

    std::cout << "Valid planners are \n";
    std::for_each(valid_planners.begin(), valid_planners.end(),
                  [](std::string p)
    {
        std::cout << " - " << p << "\n";
    });
    std::cout.flush();

    throw std::runtime_error("Planner type '" + planner_type + "' not valid!");
}
