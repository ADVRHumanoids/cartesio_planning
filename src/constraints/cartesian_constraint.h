#ifndef CARTESIAN_CONSTRAINT_H
#define CARTESIAN_CONSTRAINT_H

#include <ompl/base/Constraint.h>
#include <matlogger2/matlogger2.h>

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

    bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

    void reset();
    
    void flushLogger();

private:

    PositionCartesianSolver::Ptr _ik_solver;
    XBot::MatLogger2::Ptr _logger;

};

} } }

#endif // CARTESIAN_CONSTRAINT_H
