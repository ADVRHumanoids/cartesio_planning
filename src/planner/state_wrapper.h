#ifndef __CARTESIO_STATE_WRAPPER_H__
#define __CARTESIO_STATE_WRAPPER_H__

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


namespace XBot { namespace Cartesian { namespace Planning {

class StateWrapper
{
public:
    StateWrapper(const bool is_state_space_constrained, const unsigned int size):
        _is_constrained(is_state_space_constrained),
        _size(size){}

    bool setState(ompl::base::ScopedState<> state, const Eigen::VectorXd& value);
    bool getState(const ompl::base::ScopedState<> state, Eigen::VectorXd& value);

private:
    bool _is_constrained;
    unsigned int _size;
};

}
}
}

#endif
