#ifndef CARTESIO_PLANNING_HXX
#define CARTESIO_PLANNING_HXX

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
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
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

#include <ompl/control/planners/rrt/RRT.h>

#include "state_space.hxx"
#include <cartesio_planning/cartesio_planning.h>

namespace XBot::Cartesian::Planning {

class Planner::Impl
{

public:

    Impl(StateSpace::ConstPtr space,
         YAML::Node options);

    bool solve(Eigen::VectorXd qstart,
               Eigen::VectorXd qgoal,
               double timeout,
               std::string planner_type);

    Eigen::MatrixXd getSolutionPath(bool simplify = false, double timeout = -1) const;

private:

    ompl::base::PlannerPtr make_planner(std::string planner_type);

    void configure_RRTstar();

    StateSpace::ConstPtr _ss;

    std::shared_ptr<ompl::base::SpaceInformation> _space_info;

    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;

    YAML::Node _options;

    std::shared_ptr<ompl::base::Planner> _planner;

    bool _verbose;

};


}

#endif // CARTESIO_PLANNING_HXX
