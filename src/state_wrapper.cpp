#include "state_wrapper.h"

using namespace XBot::Cartesian::Planning;

StateWrapper::StateWrapper(StateSpaceType state_space_type,
                           int size):
    _state_space_type(state_space_type),
    _size(size)
{

}

void StateWrapper::setState(ompl::base::State * state,
                            const Eigen::VectorXd& value) const
{
    if(value.size() != _size)
    {
        throw std::out_of_range("Value size does not match state space dimension");
    }

    if(_state_space_type == StateSpaceType::CONSTRAINED) //state space is constrained
    {
        state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(value);
    }
    else if (_state_space_type == StateSpaceType::REALVECTOR) //state space is a real vector
    {
        Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;
    }
    else //SE2 state space
    {
        state->as<ompl::base::SE2StateSpace::StateType>()->setXY(value(0), value(1));
        state->as<ompl::base::SE2StateSpace::StateType>()->setYaw(value(2));
    }

}

void StateWrapper::getState(const ompl::base::State * state,
                            Eigen::VectorXd& value) const
{

    if(_state_space_type == StateSpaceType::CONSTRAINED)
    {
        value = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
    }
    else if (_state_space_type == StateSpaceType::REALVECTOR)
    {
        value = Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size);
    }
    else
    {
        value.resize(_size);
        value << state->as<ompl::base::SE2StateSpace::StateType>()->getX(),
                 state->as<ompl::base::SE2StateSpace::StateType>()->getY(),
                 state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
    }
}

StateWrapper::StateSpaceType StateWrapper::getStateSpaceType() 
{
    return _state_space_type;
}
