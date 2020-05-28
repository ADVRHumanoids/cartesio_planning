#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>

#include "state_wrapper.h"

namespace XBot { namespace Cartesian { namespace Planning { namespace Propagators{

class stepPropagator : public ompl::control::StatePropagator {
public:
    stepPropagator (ompl::control::SpaceInformationPtr si,
                    std::shared_ptr<StateWrapper> sw,
                    const ompl::base::ConstraintPtr manifold = NULL):
    ompl::control::StatePropagator(si),
    _sw(sw),
    _si(si),
    _manifold(manifold)
    {
        _start_state = _si->allocState();
    }
    
    virtual void propagate(const ompl::base::State* start,
                           const ompl::control::Control* control, 
                           const double duration,
                           ompl::base::State* result) const override
    {
        // Insert a new state in the std::map if not already in there
        _si->getStateSpace()->copyState(_start_state, start); 
        
        int foot_sel = control->as<ompl::control::CompoundControlSpace::ControlType>()->as<ompl::control::DiscreteControlSpace::ControlType>(0)->value;
        auto step_size = control->as<ompl::control::CompoundControlSpace::ControlType>()->as<ompl::control::RealVectorControlSpace::ControlType>(1)->values;
        
        _si->getStateSpace()->copyState(result, start);
        
        // Extra action for Centauro: move all the wheel together
        if (foot_sel == 3 || foot_sel == 4)
        {
            for (int i = 0; i < 4; i++)
            {
                auto moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(i);
                
                Eigen::VectorXd X, U;
                _sw->getState(moved_foot, X);
                U = Eigen::VectorXd::Map(step_size, 2);
                X += U * duration;
                
                _sw->setState(moved_foot, X);
            }
        }
        
        else if (foot_sel == 1)
        {
            auto moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
            
            Eigen::VectorXd X, U;
            _sw->getState(moved_foot, X);
            U = Eigen::VectorXd::Map(step_size, 2);
            
            X += U * duration;
            
            _sw->setState(moved_foot, X);
            
            moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(1);
            _sw->getState(moved_foot, X);
            
            X(0) += U(0) * duration;
            X(1) -= U(1) * duration;
            
            _sw->setState(moved_foot, X);
        }
        
        else if (foot_sel == 2)
        {
            auto moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(2);
            
            Eigen::VectorXd X, U;
            _sw->getState(moved_foot, X);
            U = Eigen::VectorXd::Map(step_size, 2);
            
            X += U * duration;
            
            _sw->setState(moved_foot, X);
            
            moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(3);
            _sw->getState(moved_foot, X);
            
            X(0) += U(0) * duration;
            X(1) -= U(1) * duration;
            
            _sw->setState(moved_foot, X);
        }
        
//         else
//         {
//             // Select the swing foot and cast it to the desired value
//             ompl::base::State* moved_foot;
//             if (_sw->getStateSpaceType() == StateWrapper::StateSpaceType::REALVECTOR)
//                 moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(foot_sel - 1);
//             else if (_sw->getStateSpaceType() == StateWrapper::StateSpaceType::SE2SPACE)
//                 moved_foot = result->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::SE2StateSpace::StateType>(foot_sel - 1);
//             
//             // Propagate
//             Eigen::VectorXd X, U;
//             _sw->getState(moved_foot, X);
//             if (_sw->getStateSpaceType() == StateWrapper::StateSpaceType::REALVECTOR)
//                 U = Eigen::VectorXd::Map(step_size, 2);
//             else if (_sw->getStateSpaceType() == StateWrapper::StateSpaceType::SE2SPACE)
//                 U = Eigen::VectorXd::Map(step_size, 3);
//             X += U * duration;
//             
//             _sw->setState(moved_foot, X);
//         }
    }
    
    const ompl::base::State* getStartState()
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