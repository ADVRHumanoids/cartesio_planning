#ifndef CONSTRAINT_HXX
#define CONSTRAINT_HXX

#include <cartesio_planning/constraint.h>
#include "state_space.hxx"
#include "parse_utils.hxx"

namespace XBot::Cartesian::Planning
{


class ConstraintWrapper : public ompl::base::Constraint
{

public:

    ConstraintWrapper(int ambient_dim,
                      ::XBot::Cartesian::Planning::Constraint::Ptr constraint);

    void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Eigen::Ref<Eigen::VectorXd> out) const override;

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Eigen::Ref<Eigen::MatrixXd> out) const override;
private:

    ::XBot::Cartesian::Planning::Constraint::Ptr _c;
};


class Constraint::Impl
{

public:

    Impl(Constraint& api,
         StateSpace::ConstPtr ss,
         YAML::Node options):
        _api(api),
        _ss(ss),
        _options(options)
    {
        _qneutral = _ss->neutral();
    }

    void bind(StateSpace::ConstPtr ss);

    void reset();

    int constraintSize();

    Eigen::VectorXd value(const Eigen::VectorXd& q) const;

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;

    Eigen::VectorXd sample() const;

    std::shared_ptr<ompl::base::AtlasStateSpaceNE> getAtlas() const;

    mutable ompl::base::StateSamplerPtr sampler;
    mutable ompl::base::ScopedStatePtr state;

private:

    Constraint& _api;

    YAML::Node _options;

    std::shared_ptr<ompl::base::AtlasStateSpaceNE> _atlas;

    StateSpace::ConstPtr _ss;

    std::vector<int> _eq_idx;

    Eigen::VectorXd _qmin, _qmax, _qneutral;

};


}

#endif // CONSTRAINT_HXX
