#include "state_wrapper.h"

using namespace XBot::Cartesian::Planning;

bool StateWrapper::setState(ompl::base::State* state, const Eigen::VectorXd& value)
{
    if(value.size() != _size)
        return false;

    if(_is_constrained) //state space is constrained
        state->as<ompl::base::ConstrainedStateSpace::StateType>()->copy(value);
    else //state space is not constrained
        Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size) = value;

    return true;
}

bool StateWrapper::getState(const ompl::base::State* state, Eigen::VectorXd& value)
{
    if(value.size() != _size)
        return false;

    if(_is_constrained)
        value = *state->as<ompl::base::ConstrainedStateSpace::StateType>();
    else
        value = Eigen::VectorXd::Map(state->as<ompl::base::RealVectorStateSpace::StateType>()->values, _size);

    return true;
}
