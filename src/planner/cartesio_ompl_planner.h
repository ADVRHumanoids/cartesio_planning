#ifndef __CARTESIO_PLANNING_OMPL_PLANNER_H__
#define __CARTESIO_PLANNING_OMPL_PLANNER_H__

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
#include <ompl/base/Constraint.h>

#include <Eigen/Dense>

#include "../state_wrapper.h"


namespace XBot { namespace Cartesian { namespace Planning {

class OmplPlanner
{

public:

    typedef std::function<bool(const Eigen::VectorXd&)> StateValidityPredicate;

    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max);

    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max,
                ompl::base::ConstraintPtr constraint);

    void setStateValidityPredicate(StateValidityPredicate svc);

    void setStartAndGoalStates(const Eigen::VectorXd& start,
                               const Eigen::VectorXd& goal);

    void print();

    bool solve(double timeout);

    ompl::base::PlannerStatus getPlannerStatus() const;

    std::vector<Eigen::VectorXd> getSolutionPath() const;

private:

    ompl::base::RealVectorBounds _bounds;
    std::shared_ptr<ompl::base::RealVectorStateSpace> _space;
    std::shared_ptr<ompl::base::SpaceInformation> _space_info;
    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;
    std::shared_ptr<ompl::base::Planner> _planner;
    ompl::base::PlannerStatus _solved;

    std::shared_ptr<ompl::base::ProjectedStateSpace> _css;
    std::shared_ptr<ompl::base::ConstrainedSpaceInformation> _csi;


    unsigned int _size;

    void setBounds(const Eigen::VectorXd& bounds_min,
                   const Eigen::VectorXd& bounds_max);
    void setConstraint(ompl::base::ConstraintPtr constraint);

    std::shared_ptr<StateWrapper> _sw;

};

}
}
}

#endif
