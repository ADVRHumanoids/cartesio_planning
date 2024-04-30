#include <cartesio_planning/state_validity_checker.h>

using namespace XBot::Cartesian::Planning;

StateValidityChecker::StateValidityChecker(StateSpace::ConstPtr space, std::string name, int substate_idx):
    _space(space), _name(name), _idx(substate_idx)
{

}

const std::string &StateValidityChecker::getName() const
{
    return _name;
}

int StateValidityChecker::getSubstateIndex() const
{
    return _idx;
}

void StateValidityChecker::printInvalidStateInformation(std::ostream &os) const
{

}

StateValidityChecker::~StateValidityChecker()
{

}

StateValidityCheckerFunction::StateValidityCheckerFunction(StateSpace::ConstPtr space,
                                                           std::string name,
                                                           std::function<bool (const Eigen::VectorXd &)> fun,
                                                           int substate_idx):
    StateValidityChecker(space, name, substate_idx),
    _fun(fun)
{

}

bool StateValidityCheckerFunction::checkValid(const Eigen::VectorXd &q,
                                              std::optional<Eigen::VectorXd> &qnear) const
{
    qnear.reset();

    return _fun(q);
}
