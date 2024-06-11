#ifndef CARTESIANCONSTRAINT_H
#define CARTESIANCONSTRAINT_H

#include "../state_space.h"

#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace XBot::Cartesian::Planning
{

class CartesianConstraint : public Constraint
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(CartesianConstraint)

    CartesianConstraint(CartesianInterfaceImpl::Ptr ci,
                        YAML::Node options = YAML::Node());

    int constraintSize() const override;

    void update(const Eigen::VectorXd &q) const;

    Eigen::VectorXd value(const Eigen::VectorXd &q) const override;

    Eigen::MatrixXd jacobian(const Eigen::VectorXd &q) const override;

    bool refine(Eigen::VectorXd& q) const override;

private:

    CartesianInterfaceImpl::Ptr _ci;

    int _nc;

    mutable Eigen::VectorXd _old_q;

};
}

#endif // CARTESIANCONSTRAINT_H
