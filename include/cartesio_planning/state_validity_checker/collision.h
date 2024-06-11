#ifndef CARTESIO_PLANNING_COLLISION_H
#define CARTESIO_PLANNING_COLLISION_H

#include <xbot2_interface/collision.h>

#include "../state_validity_checker.h"

namespace XBot::Cartesian::Planning {

class CollisionValidityChecker : public StateValidityChecker
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(CollisionValidityChecker)

    CollisionValidityChecker(StateSpace::ConstPtr space,
                             XBot::Collision::CollisionModel::Ptr collision_model,
                             std::optional<std::string> id = std::nullopt,
                             int substate_idx = 0);

    void setIncludeEnvironment(bool flag);

    void setThreshold(double threshold);

    bool checkValid(const Eigen::VectorXd &q, std::optional<Eigen::VectorXd> &qnear) const override;

    void printInvalidStateInformation(std::ostream &os) const override;

    typedef std::function<void(const Collision::CollisionModel::LinkPairVector& lpv)> Callback;

    void setCallback(Callback cb);

    ~CollisionValidityChecker();

private:

    class Impl;

    std::unique_ptr<Impl> impl;
};


}

#endif // COLLISION_H
