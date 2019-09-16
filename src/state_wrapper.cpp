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
    else //state space is not constrained
    {
        Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;
    }

}

void StateWrapper::getState(const ompl::base::State * state,
                            Eigen::VectorXd& value) const
{

    if(_state_space_type == StateSpaceType::CONSTRAINED)
    {
        value = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
    }
    else
    {
        value = Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size);
    }
}
