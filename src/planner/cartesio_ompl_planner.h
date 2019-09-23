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

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

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

    void setYaml(YAML::Node options);

    void setStateValidityPredicate(StateValidityPredicate svc);

    void setStartAndGoalStates(const Eigen::VectorXd& start,
                               const Eigen::VectorXd& goal);

    void setStartAndGoalStates(const Eigen::VectorXd& start,
                               ompl::base::GoalPtr goal);

    void print(std::ostream &out = std::cout);

    bool solve(const double timeout, const std::string& planner_type);

    ompl::base::PlannerStatus getPlannerStatus() const;

    std::vector<Eigen::VectorXd> getSolutionPath() const;

    ompl::base::SpaceInformationPtr getSpaceInfo() const;

    StateWrapper getStateWrapper() const;

private:

    typedef std::shared_ptr<ompl::base::Planner> PlannerPtr;

    void setBounds(const Eigen::VectorXd& bounds_min,
                   const Eigen::VectorXd& bounds_max);

    void setUpProblemDefinition();

    PlannerPtr plannerFactory(const std::string& planner_type);
    PlannerPtr makeRRTstar();



    ompl::base::RealVectorBounds _bounds;
    std::shared_ptr<ompl::base::RealVectorStateSpace> _ambient_space;
    std::shared_ptr<ompl::base::SpaceInformation> _space_info;

    std::shared_ptr<ompl::base::ProblemDefinition> _pdef;
    std::shared_ptr<ompl::base::Planner> _planner;
    ompl::base::PlannerStatus _solved;

    std::shared_ptr<ompl::base::StateSpace> _space;


    unsigned int _size;



    std::shared_ptr<StateWrapper> _sw;

    YAML::Node _options;

};

}
                                     }
               }

#endif
