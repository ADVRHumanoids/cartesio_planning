#ifndef __CARTESIO_PLANNING_OMPL_PLANNER_H__
#define __CARTESIO_PLANNING_OMPL_PLANNER_H__

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "../state_wrapper.h"


namespace XBot { namespace Cartesian { namespace Planning {


class OmplPlanner
{

public:

    typedef std::function<bool(const Eigen::VectorXd&)> StateValidityPredicate;
    typedef std::shared_ptr<OmplPlanner> Ptr;

    OmplPlanner(const OmplPlanner&) = delete;
    OmplPlanner& operator=(const OmplPlanner&) = delete;


    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max,
                YAML::Node options);

    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max,
                ompl::base::ConstraintPtr constraint,
                YAML::Node options);

    // constructors for planning with controls
    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max,
                const Eigen::VectorXd& control_min,
                const Eigen::VectorXd& control_max,
                YAML::Node options);

    OmplPlanner(const Eigen::VectorXd& bounds_min,
                const Eigen::VectorXd& bounds_max,
                const Eigen::VectorXd& control_min,
                const Eigen::VectorXd& control_max,
                ompl::base::ConstraintPtr constraint,
                YAML::Node options);

    void setStateValidityPredicate(StateValidityPredicate svc);

    void setStartAndGoalStates(const Eigen::VectorXd& start,
                               const Eigen::VectorXd& goal,
                               const double threshold = std::numeric_limits<double>::epsilon());

    void setStartAndGoalStates(const Eigen::VectorXd& start,
                               std::shared_ptr<ompl::base::GoalSampleableRegion> goal,
                               const double threshold = std::numeric_limits<double>::epsilon());

    void print(std::ostream &out = std::cout);

    bool solve(const double timeout, const std::string& planner_type);

    ompl::base::PlannerStatus getPlannerStatus() const;

    std::vector<Eigen::VectorXd> getSolutionPath() const;

    ompl::base::SpaceInformationPtr getSpaceInfo() const;
    ompl::control::SpaceInformationPtr getControlSpaceInfo() const;

    void getBounds(Eigen::VectorXd& qmin, Eigen::VectorXd& qmax) const;

    void getControlBounds(Eigen::VectorXd& control_min, Eigen::VectorXd& control_max)  const;

    StateWrapper getStateWrapper() const;


private:

    void set_bounds(const Eigen::VectorXd& bounds_min,
                   const Eigen::VectorXd& bounds_max);

    void set_control_bounds(const Eigen::VectorXd& control_min,
                            const Eigen::VectorXd& control_max);

    void setup_problem_definition(std::shared_ptr<ompl::base::SpaceInformation> space_info);

    ompl::base::PlannerPtr make_planner(const std::string& planner_type);
    ompl::base::PlannerPtr make_RRTstar();

    ompl::base::StateSpacePtr make_constrained_space();
    ompl::base::StateSpacePtr make_atlas_space();
    ompl::base::StateSpacePtr make_tangent_bundle();
    ompl::base::StateSpacePtr make_projected_space();

    ompl::base::ConstraintPtr _constraint;

    ompl::base::RealVectorBounds _sbounds; //state bounds

    std::shared_ptr<ompl::base::RealVectorStateSpace> _ambient_space;
    std::shared_ptr<ompl::base::SpaceInformation> _space_info;

    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;
    std::shared_ptr<ompl::base::Planner> _planner;
    ompl::base::PlannerStatus _solved;

    std::shared_ptr<ompl::base::StateSpace> _space;

    std::shared_ptr<ompl::base::RealVectorBounds> _cbounds; //control bounds
    std::shared_ptr<ompl::control::RealVectorControlSpace> _cspace;
    std::shared_ptr<ompl::control::SpaceInformation> _cspace_info;

    std::function<void(void)> _on_reset_space;
    std::function<void(const ompl::base::State* start,
                       const ompl::base::State* goal)> _on_set_start_goal;

    unsigned int _size;

    std::shared_ptr<StateWrapper> _sw;

    YAML::Node _options;


};

}
                                     }
               }

#endif
