#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>

#include "state_wrapper.h"

namespace XBot { namespace Cartesian { namespace Planning { namespace Propagators{

class stepPropagatorSE2 : public ompl::control::StatePropagator {
public:
    stepPropagatorSE2 (ompl::control::SpaceInformationPtr si,
                    std::shared_ptr<StateWrapper> sw,
                    const ompl::base::ConstraintPtr manifold = NULL):
    ompl::control::StatePropagator(si),
    _sw(sw),
    _si(si),
    _manifold(manifold)
    {}
    
    virtual void propagate(const ompl::base::State* start,
                           const ompl::control::Control* control, 
                           const double duration,
                           ompl::base::State* result) const override
    {
        // Insert a new state in the std::map if not already in there
        _si->getStateSpace()->copyState(_start_state, start);  
        
        // Cast state and control to the desired type
        int foot_sel = control->as<ompl::control::CompoundControlSpace::ControlType>()->as<ompl::control::DiscreteControlSpace::ControlType>(0)->value;
        auto step_size = control->as<ompl::control::CompoundControlSpace::ControlType>()->as<ompl::control::RealVectorControlSpace::ControlType>(1)->values;
        
        _si->getStateSpace()->copyState(result, start);
        
        auto moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(foot_sel - 1);
        
        // Propagate
        moved_foot->setXY(moved_foot->getX() + duration*step_size[0], moved_foot->getY() + duration*step_size[1]);
        moved_foot->setYaw(moved_foot->getYaw() + duration*step_size[2]);      
    }
    
    ompl::base::State* getStartState()
    {
        return _start_state;
    }

private:
    std::shared_ptr<StateWrapper> _sw;
    ompl::base::ConstraintPtr _manifold;
    ompl::control::SpaceInformationPtr _si;
    ompl::base::State* _start_state;
};
} } } }