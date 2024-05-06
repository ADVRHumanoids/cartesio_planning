#include <cartesio_planning/cartesio_planning.h>

#include "impl/cartesio_planning.hxx"
#include "impl/parse_utils.hxx"
#include "impl/profiling.hxx"

using namespace XBot::Cartesian::Planning;

Planner::Planner(StateSpace::ConstPtr space, YAML::Node options)
{
    impl = std::make_unique<Impl>(space, options);
}

bool Planner::addStateValidityChecker(StateValidityChecker::ConstPtr svc)
{
    return impl->addStateValidityChecker(svc);
}

bool Planner::checkValid(const Eigen::VectorXd &q,
                         std::vector<std::string> *failed_checks,
                         std::ostream &report_os) const
{
    return impl->checkValid(q, true, report_os, failed_checks);
}

bool Planner::solve(Eigen::VectorXd qstart, Eigen::VectorXd qgoal, double timeout, std::string planner_type)
{
    return impl->solve(qstart, qgoal, timeout, planner_type);
}

Eigen::MatrixXd Planner::getSolutionPath(bool simplify, double timeout) const
{
    return impl->getSolutionPath(simplify, timeout);
}

Planner::~Planner()
{

}

Planner::Impl::Impl(StateSpace::ConstPtr space, YAML::Node options):
    _ss(space), _options(options), _verbose(false)
{

}

bool Planner::Impl::solve(Eigen::VectorXd qstart,
                          Eigen::VectorXd qgoal,
                          double timeout,
                          std::string planner_type)
{
    // create everything
    _space_info = std::make_shared<ompl::base::SpaceInformation>(_ss->getImpl().getStateSpace());

    _space_info->setStateValidityChecker(

        [this](const ompl::base::State * state){

            return isStateValid(*state);

        });

    _pdef = std::make_shared<ompl::base::ProblemDefinition>(_space_info);

    // create requested planner
    auto planner = make_planner(planner_type);
    planner->setProblemDefinition(_pdef);
    planner->setup();

    // note: verbose
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);

    // set start and goal
    auto start = _ss->getImpl().createState();
    _ss->getImpl().setValue(*start, qstart);

    auto goal = _ss->getImpl().createState();
    _ss->getImpl().setValue(*goal, qgoal);

    // check validity
    if(!_ss->getImpl().getStateSpace()->satisfiesBounds(start))
    {
        throw std::runtime_error("start state is out of bounds");
    }

    if(!isStateValid(*start, true))
    {
        throw std::runtime_error("start state is invalid");
    }

    if(!_ss->getImpl().getStateSpace()->satisfiesBounds(goal))
    {
        throw std::runtime_error("goal state is out of bounds");
    }

    if(!isStateValid(*goal, true))
    {
        throw std::runtime_error("goal state is invalid");
    }

    // clear previous start/goal and solution
    planner->clearQuery();
    _pdef->clearSolutionPaths();

    // set start and goal
    double threshold = 0.0;
    _pdef->setStartAndGoalStates(start, goal, threshold);

    // clear profiling
    ProfilingData::instance().reset();

    // solve
    TIC(planner);
    auto solved_flag = _planner->solve(timeout);
    double solve_time = TOC(planner);

    // print profiling
    ProfilingData::instance().print(std::cout, solve_time);

    return solved_flag == ompl::base::PlannerStatus::EXACT_SOLUTION;

}

bool Planner::Impl::addStateValidityChecker(StateValidityChecker::ConstPtr svc)
{
    if(_svc_map.count(svc->getName()))
    {
        return false;
    }

    _svc_map[svc->getName()] = svc;

    return true;
}

bool Planner::Impl::isStateValid(const ompl::base::State &state,
                                 bool force_verbose,
                                 std::ostream& os,
                                 std::vector<std::string> * failed_checks) const
{

    Eigen::VectorXd q = _ss->getImpl().getValue(state);

    return checkValid(q, force_verbose, os, failed_checks);

}

bool Planner::Impl::checkValid(const Eigen::VectorXd &q,
                               bool force_verbose,
                               std::ostream &os,
                               std::vector<std::string> *failed_checks) const
{
    TIKTOK(state_validity_check);

    _ss->getImpl().updateModelState(q);

    std::optional<Eigen::VectorXd> qnear;

    for(const auto& [cname, c] : _svc_map)
    {
        qnear.reset();

        if(!c->checkValid(q, qnear))
        {
            if(force_verbose || _verbose)
            {
                os << "validity check '" << cname << "' failed: ";
                c->printInvalidStateInformation(os);
                os << "\n";
            }

            if(failed_checks)
            {
                failed_checks->push_back(cname);
            }

            return false;
        }
    }

    TIKTOK(state_valid);

    return true;
}



Eigen::MatrixXd Planner::Impl::getSolutionPath(bool simplify, double timeout) const
{
    auto * geom_path = _pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();

    if(!geom_path)
    {
        throw std::runtime_error("the planner contains no plan");

    }

    // simplify
    if(simplify)
    {
        ompl::geometric::PathSimplifier ps(_space_info);

        bool simplify_ok = ps.simplify(*geom_path,
                                       timeout < 0 ? std::numeric_limits<double>::max() : timeout,
                                       true);

        if(!simplify_ok)
        {
            throw std::runtime_error("failed to simplify path");
        }
    }


    Eigen::MatrixXd path(_ss->getNq(), geom_path->getStateCount());

    for(int i = 0; i < geom_path->getStateCount(); i++)
    {
        path.col(i) = _ss->getImpl().getValue(*geom_path->getState(i));
    }

    return path;
}


ompl::base::PlannerPtr Planner::Impl::make_planner(std::string planner_type)
{
    using PlannerFactory = std::function<ompl::base::PlannerPtr(std::shared_ptr<ompl::base::SpaceInformation>)>;
    using SpaceInfoPtr = std::shared_ptr<ompl::base::SpaceInformation>;

    std::map<std::string, PlannerFactory> planner_factory = {

        {"RRT", std::make_shared<ompl::geometric::RRT, SpaceInfoPtr>},
        {"RRTConnect", std::make_shared<ompl::geometric::RRTConnect, SpaceInfoPtr>},
        {"RRTstar", std::make_shared<ompl::geometric::RRTstar, SpaceInfoPtr>},
        {"PRM", std::make_shared<ompl::geometric::PRM, SpaceInfoPtr>},
        {"PRMstar", std::make_shared<ompl::geometric::PRMstar, SpaceInfoPtr>}
    };

    std::map<std::string, std::function<void()>> config_functions = {
        {"RRTstar", std::bind(&Planner::Impl::configure_RRTstar, this)}
    };

    _planner.reset();

    try
    {
        // construct planner and save it
        _planner = planner_factory.at(planner_type)(_space_info);

        // configure planner
        if(config_functions.count(planner_type) > 0)
        {
            config_functions.at(planner_type)();
        }

        return _planner;
    }
    catch(std::out_of_range& e)
    {
        std::cerr << "planner type undefined, available types are: ";

        for(auto [ptype, pfun] : planner_factory)
        {
            std::cerr << ptype << ", ";
        }

        throw;
    }
}

void Planner::Impl::configure_RRTstar()
{
    if(!_options || !_options["RRTstar"])
    {
        std::cout << "No options detected" << std::endl;
    }

    auto opt = _options["RRTstar"];

    auto planner = std::dynamic_pointer_cast<ompl::geometric::RRTstar>(_planner);

    PLANNER_PARSE_OPTION(GoalBias, double);
    PLANNER_PARSE_OPTION(Range, int);
    PLANNER_PARSE_OPTION(KNearest, bool);
}
