#include "state_wrapper.h"
#include <ompl/base/State.h>
#include <ompl/control/Control.h>
#include <ompl/control/StatePropagator.h>

namespace XBot { namespace Cartesian { namespace Planning { namespace Propagators{

class RK1 : public ompl::control::StatePropagator{
public:
    RK1(ompl::control::SpaceInformation* si, StateWrapper& sw):
        ompl::control::StatePropagator(si),
        _sw(sw)
    {
        _control_size = si->getStateDimension();
    }

    virtual void propagate(const ompl::base::State *start, const ompl::control::Control *control,
                   const double duration, ompl::base::State *result) const override
    {
        Eigen::VectorXd X, U;

        _sw.getState(start, X);

        U = Eigen::VectorXd::Map(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values, _control_size);

        X = X + duration*U;

        _sw.setState(result, X);
    }

private:
    StateWrapper& _sw;
    int _control_size;

};

} } } }
