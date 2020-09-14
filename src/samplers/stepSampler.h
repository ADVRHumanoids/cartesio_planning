#include <ompl/control/ControlSampler.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/base/State.h>
#include <ompl/control/Control.h>

namespace XBot { namespace Cartesian { namespace Planning {

class stepSampler : public ompl::control::ControlSampler {
public:
    stepSampler (const ompl::control::ControlSpace* cspace):
    ompl::control::ControlSampler(cspace) {}   
    
    void sample(ompl::control::Control* control) override
    {
        control->as<ompl::control::DiscreteControlSpace::ControlType>()->value = rng_.uniformInt(
                space_->as<ompl::control::DiscreteControlSpace>()->getLowerBound(), space_->as<ompl::control::DiscreteControlSpace>()->getUpperBound());
    }
    
    void sample(ompl::control::Control* control, const ompl::base::State* state) override
    {
        sample(control);
    }
    
    void sampleNext(ompl::control::Control *control, const ompl::control::Control *previous) override
    {
        do 
        {
            sample(control);
        }
        while (control->as<ompl::control::DiscreteControlSpace::ControlType>()->value == previous->as<ompl::control::DiscreteControlSpace::ControlType>()->value);
    } 
    
    void sampleNext(ompl::control::Control* control, const ompl::control::Control* previous, const ompl::base::State* state) override
    {
        sampleNext(control,previous);
    }
};    
} } }