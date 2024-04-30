#ifndef STATE_VALIDITY_CHECKER_H
#define STATE_VALIDITY_CHECKER_H

#include "common/types.h"
#include "state_space.h"

namespace XBot::Cartesian::Planning {

/**
 * @brief The StateValidityChecker class
 */
class StateValidityChecker
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(StateValidityChecker);

    StateValidityChecker(StateSpace::ConstPtr space, std::string name, int substate_idx = 0);

    const std::string& getName() const;

    int getSubstateIndex() const;

    virtual bool checkValid(const Eigen::VectorXd &q,
                            std::optional<Eigen::VectorXd>& qnear) const = 0;

    virtual void printInvalidStateInformation(std::ostream& os) const;

    virtual ~StateValidityChecker();

protected:

    StateSpace::ConstPtr _space;
    std::string _name;
    int _idx;

};


/**
 * @brief The StateValidityCheckerFunction class
 */
class StateValidityCheckerFunction : public StateValidityChecker
{

public:

    StateValidityCheckerFunction(StateSpace::ConstPtr space,
                                 std::string name,
                                 std::function<bool(const Eigen::VectorXd &q)> fun,
                                 int substate_idx = -1);

    bool checkValid(const Eigen::VectorXd &q,
                    std::optional<Eigen::VectorXd>& qnear) const override;


protected:

    std::function<bool(const Eigen::VectorXd &q)> _fun;

};


}

#endif // STATE_VALIDITY_CHECKER_H
