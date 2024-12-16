#include <cartesio_planning/constraints/contact_constraint.h>

#include <xbot2_interface/common/utils.h>


using namespace XBot::Cartesian::Planning;


ContactConstraint::ContactConstraint(std::shared_ptr<const StateSpace> space,
                                     XBot::ModelInterface::Ptr model,
                                     std::map<std::string, std::vector<int> > contact_map,
                                     YAML::Node options):
    Constraint(space, options),
    _model(model)
{
    int csize = 0;

    for(auto [cname, idx] : contact_map)
    {
        csize += idx.size();

        auto valid_idx = [](int i)
        {
            return i >= 0 && i < 6;
        };


        if(!std::all_of(idx.begin(), idx.end(), valid_idx))
        {
            throw std::invalid_argument("invalid index vector (i < 0 or i >= 6)");
        }

        _contact_map[cname].indices = idx;
        _contact_map[cname].T = model->getPose(cname);
    }

    _val.setZero(csize);

    _J.setZero(csize, _model->getNv());
}

int ContactConstraint::_constraintSize() const
{
    return _val.size();
}

void ContactConstraint::resetContactPose()
{
    for(auto& [cname, c] : _contact_map)
    {
        c.T = _model->getPose(cname);
    }
}

void ContactConstraint::setContactPose(std::string name, Eigen::Affine3d T)
{
    _contact_map.at(name).T = T;
}

void ContactConstraint::update(const Eigen::VectorXd &q) const
{
    if(_old_q.size() > 0 && q.cwiseEqual(_old_q).all())
    {
        return;
    }

    _model->setJointPosition(q);

    _model->update();

    _old_q = q;
}

Eigen::VectorXd ContactConstraint::_value(const Eigen::VectorXd &q) const
{
    update(q);

    int k = 0;

    for(const auto& [cname, c] : _contact_map)
    {
        Eigen::Affine3d T = _model->getPose(cname);

        auto err = XBot::Utils::computePoseError(c.T, T);

        // Utils::rotate(err, T.linear().transpose());

        for(auto i : c.indices)
        {
            _val[k++] = err[i];
        }
    }

    return _val;
}

Eigen::MatrixXd ContactConstraint::_jacobian(const Eigen::VectorXd &q) const
{
    update(q);

    int k = 0;

    for(const auto& [cname, c] : _contact_map)
    {
        Eigen::Affine3d T = _model->getPose(cname);

        Eigen::MatrixXd J = _model->getJacobian(cname);

        Eigen::MatrixXd Jrot(J.rows(), J.cols());

        // Utils::rotate(J, T.linear().transpose(), Jrot);

        Jrot = J;

        for(auto i : c.indices)
        {
            _J.row(k++) = -Jrot.row(i);
        }
    }

    return _J;
}

bool ContactConstraint::refine(Eigen::VectorXd &q) const
{
    return Constraint::refine(q);
}
