#ifndef CARTESIAN_CONSTRAINT_H
#define CARTESIAN_CONSTRAINT_H

#include <ompl/base/Constraint.h>

#include "../ik/position_ik_solver.h"

namespace XBot { namespace Cartesian { namespace Planning {

class CartesianConstraint : public ompl::base::Constraint
{

public: // Constraint interface

    typedef std::shared_ptr<CartesianConstraint> Ptr;

    CartesianConstraint(PositionCartesianSolver::Ptr ik_solver);

    void function(const Eigen::Ref<const Eigen::VectorXd>& x,
                  Eigen::Ref<Eigen::VectorXd> out) const override;

    void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                  Eigen::Ref<Eigen::MatrixXd> out) const override;

    bool project(Eigen::Ref<Eigen::VectorXd> x) const;

    void reset();

private:

    PositionCartesianSolver::Ptr _ik_solver;

};

} } }

#endif // CARTESIAN_CONSTRAINT_H
