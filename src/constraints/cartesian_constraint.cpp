#include "cartesian_constraint.h"

namespace XBot { namespace Cartesian { namespace Planning {

CartesianConstraint::CartesianConstraint(PositionCartesianSolver::Ptr ik_solver):
    ompl::base::Constraint(ik_solver->getModel()->getJointNum(), // ambient space dim
                           ik_solver->getSize()),                // num constraints
    _ik_solver(ik_solver)
{
    setTolerance(_ik_solver->getErrorThreshold());
//     _logger = XBot::MatLogger2::MakeLogger("/home/luca/my_log/my_log");
}

void CartesianConstraint::function(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::Ref<Eigen::VectorXd> out) const
{
    auto model = _ik_solver->getModel();

    model->setJointPosition(x);
    model->update();

    Eigen::VectorXd f;
    _ik_solver->getError(f);

    out = f;
}

void CartesianConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                                   Eigen::Ref<Eigen::MatrixXd> out) const
{
    auto model = _ik_solver->getModel();

    model->setJointPosition(x);
    model->update();

    Eigen::MatrixXd J;
    _ik_solver->getJacobian(J);

    out = J;
}

bool CartesianConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const
{
//     return Constraint::project(x);
    
//     Eigen::Vector3d x_bef {x(0), x(1), x(2)};
//     _logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
//     _logger->add("vertices_before", x_bef);
    
    auto model = _ik_solver->getModel();

    // set it to the model
    model->setJointPosition(x);
    model->update();

    if(!_ik_solver->solve())
    {
        return false;
    }

    Eigen::VectorXd q_proj;
    model->getJointPosition(q_proj);
    x = q_proj;
    
//     Eigen::Vector3d x_aft {x(0), x(1), x(2)};
//     _logger->add("vertices_after", x_aft);
//     _logger->add("bool", _ik_solver->solve());
// 
    return true;

}

void CartesianConstraint::reset()
{
    _ik_solver->reset();
}

// void CartesianConstraint::flushLogger()
// {
//     _logger.reset();
//     _logger = _logger = XBot::MatLogger2::MakeLogger("/home/luca/my_log/my_log");
// 
// }


} } }
