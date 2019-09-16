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

    enum class StateSpaceType { REALVECTOR, CONSTRAINED };

    StateWrapper(StateSpaceType state_space_type,
                 int size);

    void setState(ompl::base::State * state,
                  const Eigen::VectorXd& value) const;

    void getState(const ompl::base::State * state,
                  Eigen::VectorXd& value) const;

private:

    StateSpaceType _state_space_type;
    int _size;
};

}
}
}

#endif
