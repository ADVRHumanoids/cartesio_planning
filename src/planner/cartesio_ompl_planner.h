#ifndef _CARTESIO_OMPL_PLANNER_H_
#define _CARTESIO_OMPL_PLANNER_H_

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <Eigen/Dense>


namespace XBot { namespace Cartesian { namespace Planning {

/**
 * T has to derive from ompl::base::Planner
 */
template <typename T> class OMPLPlanner
{
public:
    OMPLPlanner(const Eigen::VectorXd& bounds_min, const Eigen::VectorXd& bounds_max);

    bool setBounds(const Eigen::VectorXd& bounds_min, const Eigen::VectorXd& bounds_max);
    void setStateValidityChecker(const ompl::base::StateValidityCheckerPtr &svc);
    void setStateValidityChecker(const ompl::base::StateValidityCheckerFn &svc);

    bool setStartAndGoalStates(ompl::base::ScopedState<> start, ompl::base::ScopedState<> goal);

    void print();

    bool solve(const unsigned int t);
    const ompl::base::PlannerStatus& getPlannerStatus() const {return _solved;}
    std::shared_ptr<ompl::base::RealVectorStateSpace> getSpace(){return _space;}

    ompl::base::PathPtr getSolutionPath();

private:
    ompl::base::RealVectorBounds _bounds;
    std::shared_ptr<ompl::base::RealVectorStateSpace> _space;
    std::shared_ptr<ompl::base::SpaceInformation> _space_info;
    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;
    std::shared_ptr<T> _planner;
    ompl::base::PlannerStatus _solved;


    unsigned int _size;

    bool setState(ompl::base::ScopedState<> state, const Eigen::VectorXd& value);

};

}
}
}

#endif
