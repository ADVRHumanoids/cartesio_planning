#include "state_wrapper.h"

using namespace XBot::Cartesian::Planning;

StateWrapper::StateWrapper(const bool is_state_space_constrained,
                           const unsigned int size):
    _is_constrained(is_state_space_constrained),
    _size(size)
{

}

void StateWrapper::setState(ompl::base::State * state,
                            const Eigen::VectorXd& value)
{
    if(value.size() != _size)
    {
        throw std::out_of_range("Value size does not match state space dimension");
    }

    if(_is_constrained) //state space is constrained
    {
        state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(value);
    }
    else //state space is not constrained7
    {
        Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;
    }

}

void StateWrapper::getState(const ompl::base::State * state,
                            Eigen::VectorXd& value)
{

    if(_is_constrained)
    {
        value = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
    }
    else
    {
        value = Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size);
    }
}
