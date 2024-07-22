#include <cartesio_planning/constraints/cartesian_constraint.h>
#include "impl/profiling.hxx"

using namespace XBot::Cartesian::Planning;

CartesianConstraint::CartesianConstraint(std::shared_ptr<const StateSpace> space,
                                         CartesianInterfaceImpl::Ptr ci,
                                         YAML::Node options):
    Constraint(space, options),
    _ci(ci)
{
    // update task error and jac
    ci->update(0, 0);

    // compute constraint size
    const auto& pb = _ci->getIkProblem();

    _nc = 0;

    for(auto& t : pb.getTask(0))
    {
        Eigen::VectorXd e;

        if(!t->getTaskError(e))
        {
            throw std::runtime_error("could not get task error from task " + t->getName());
        }

        _nc += t->getIndices().size();
    }

}

int CartesianConstraint::_constraintSize() const
{
    return _nc;
}

void CartesianConstraint::update(const Eigen::VectorXd &q) const
{
    if(_old_q.size() == 0 || _old_q.cwiseNotEqual(q).any())
    {
        // note
        // as a hack, calling update(0, 0) does not trigger the qp solver
        // we just need to update error value and jacobian here!
        _ci->getModel()->setJointPosition(q);
        _ci->getModel()->update();
        _ci->update(0, 0);
        _old_q = q;
    }

}

Eigen::VectorXd CartesianConstraint::_value(const Eigen::VectorXd &q) const
{
    update(q);

    Eigen::VectorXd ret(_nc);

    const auto& pb = _ci->getIkProblem();

    int i = 0;

    for(auto& t : pb.getTask(0))
    {
        Eigen::VectorXd e;

        if(!t->getTaskError(e))
        {
            throw std::runtime_error("could not get task error from task " + t->getName());
        }

        for(auto idx : t->getIndices())
        {
            ret(i++) = e[idx];
        }

    }

    return -ret;
}

Eigen::MatrixXd CartesianConstraint::_jacobian(
    const Eigen::VectorXd &q) const
{
    update(q);

    Eigen::MatrixXd ret(_nc, _ci->getModel()->getNv());

    const auto& pb = _ci->getIkProblem();

    int i = 0;

    for(auto& t : pb.getTask(0))
    {
        Eigen::MatrixXd J;

        if(!t->getTaskErrorJacobian(J))
        {
            throw std::runtime_error("could not get task error jacobian from task " + t->getName());
        }

        for(auto idx : t->getIndices())
        {
            ret.row(i++) = J.row(idx);
        }
    }

    return ret;
}

bool CartesianConstraint::refine(Eigen::VectorXd &q) const
{
    TIKTOK(cartesian_refine);

    auto is_valid = [this](const Eigen::VectorXd& q)
    {
        return _space->checkValid(q);
    };

    auto model = _ci->getModel();

    for(int iter = 0; iter < 100; iter++)
    {
        model->setJointPosition(q);
        model->update();
        _ci->update(0, 1.0);

        Eigen::VectorXd dq = model->getJointVelocity();

        if(dq.norm() < 1e-3)
        {
            return true;
        }

        double alpha = 1.0;

        for(;;)
        {
            auto qtmp = model->sum(q, dq*alpha);

            if(is_valid(qtmp))
            {
                q = qtmp;
                break;
            }

            alpha *= 0.5;

            if(alpha < 1e-2)
            {
                return true;
            }
        }
    }

    return false;
}
