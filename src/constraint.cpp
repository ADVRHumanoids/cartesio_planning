#include <cartesio_planning/constraint.h>
#include <iostream>
#include <cartesio_planning/state_space.h>

#include "impl/constraint.hxx"
#include "impl/profiling.hxx"


using namespace XBot::Cartesian::Planning;


Constraint::Constraint(std::shared_ptr<const StateSpace> space,
                       YAML::Node options):
    _space(space)
{
    impl = std::make_unique<Impl>(*this, space, options);

    reset();
}

void Constraint::bind(std::shared_ptr<const StateSpace> space)
{
    _space = space;

    impl->bind(space);
}

void Constraint::reset()
{
    impl->reset();
}

int Constraint::constraintSize() const
{
    return impl->constraintSize();
}

Eigen::VectorXd Constraint::value(const Eigen::VectorXd &q) const
{
    return impl->value(q);
}

Eigen::MatrixXd Constraint::jacobian(const Eigen::VectorXd &q) const
{
    return impl->jacobian(q);
}

bool Constraint::project(Eigen::VectorXd &q) const
{
    // TODO handle state bounds

    TIKTOK(constr_project);

    if(!_space)
    {
        throw std::runtime_error("cannot call project() on unbound constraint");
    }

    Eigen::VectorXd val = value(q);
    Eigen::VectorXd qproj_tmp;
    Eigen::VectorXd dq;
    Eigen::MatrixXd J;

    auto qneutral = _space->neutral();
    auto q_minus_n =_space->difference(q, qneutral);
    auto [qmin_minus_n, qmax_minus_n] = _space->getBounds();

    for(int k = 0; k < 100; k++)
    {
        // compute GN step
        J = jacobian(q);

        dq = J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(-val);

        // line search
        double alpha = 1.0;

        double cost = val.norm();

        if(cost < 1e-4)
        {
            return true;
        }

        for(int i = 0; i < 10; i++)
        {
            // project inside bounds
            Eigen::VectorXd adq = (alpha * dq).cwiseMin(qmax_minus_n - q_minus_n)
                                      .cwiseMax(qmin_minus_n - q_minus_n);

            // make GN step
            qproj_tmp = _space->sum(q, adq);

            // re-compute value
            val = value(qproj_tmp);

            // simplified armijo
            if(val.norm() < cost)
            {
                break;
            }

            // on failure, reduce step size
            alpha *= 0.5;

        }

        q = qproj_tmp;
    }

    return false;
}

bool Constraint::refine(Eigen::VectorXd &q) const
{
    return true;
}

bool Constraint::checkJacobian(const Eigen::VectorXd &q) const
{
    if(!_space)
    {
        throw std::runtime_error("cannot call checkJacobian() on unbound constraint");
    }

    auto J = jacobian(q);

    bool ret = true;

    for(int i = 0; i < _space->getNv(); i++)
    {
        double h = 1e-3;

        Eigen::VectorXd vplus = h*Eigen::VectorXd::Unit(_space->getNv(), i);
        Eigen::VectorXd vminus = -vplus;

        auto qplus = _space->sum(q, vplus);
        auto qminus = _space->sum(q, vminus);

        Eigen::VectorXd Ji = (value(qplus) - value(qminus))/(2*h);

        if((Ji - J.col(i)).norm() > 1e-3)
        {
            std::cerr << "Ji      = " << J.col(i).transpose() << "\n"
                      << "Ji (fd) = " << Ji.transpose() << "\n";
            ret = false;
        }

    }

    return ret;
}

Eigen::VectorXd Constraint::sample() const
{
    return impl->sample();
}

Constraint::~Constraint() {}

Constraint::Impl &Constraint::getImpl() const
{
    return *impl;
}

void Constraint::Impl::bind(StateSpace::ConstPtr ss)
{
    _ss = ss;

    auto &simpl = ss->getImpl();

    auto ompl_ss = simpl.getAmbientStateSpace();

    auto cw = std::make_shared<ConstraintWrapper>(ompl_ss->getDimension(),
                                                  _api.shared_from_this());

    namespace pl = std::placeholders;

    _atlas = std::make_shared<ompl::base::AtlasStateSpaceNE>(
        ompl_ss,
        cw,
        std::bind(&StateSpace::sum, ss.get(), pl::_1, pl::_2),
        std::bind(&StateSpace::difference, ss.get(), pl::_1, pl::_2),
        simpl.getNv());

    auto opt = _options["Atlas"];
    OBJECT_PARSE_OPTION(_atlas, Epsilon, double);
    OBJECT_PARSE_OPTION(_atlas, Rho, double);
    OBJECT_PARSE_OPTION(_atlas, Alpha, double);
    OBJECT_PARSE_OPTION(_atlas, Exploration, double);
}

void Constraint::Impl::reset()
{
    // clear anchors
    try
    {
        getAtlas()->clear();
    }
    catch(std::runtime_error&)
    {

    }

    // add constraints from equal bounds
    std::tie(_qmin, _qmax) = _ss->getBounds();

    _eq_idx.clear();

    for(int i = 0; i < _qmin.size(); i++)
    {
        if(_qmin[i] == _qmax[i])
        {
            _eq_idx.push_back(i);

            std::cout << "[Constraint::reset] " <<
                "found equality bound constraint at index " << i << "\n";
        }
    }
}

int Constraint::Impl::constraintSize()
{
    return _api._constraintSize() + _eq_idx.size();
}

Eigen::VectorXd Constraint::Impl::value(const Eigen::VectorXd &q) const
{
    Eigen::VectorXd value = _api._value(q);

    int i = value.size();

    value.conservativeResize(value.size() + _eq_idx.size());

    auto qdiff = _ss->difference(q, _qneutral);

    for(int k : _eq_idx)
    {
        value[i++] = qdiff[k] - _qmin[k];
    }

    return value;
}

Eigen::MatrixXd Constraint::Impl::jacobian(const Eigen::VectorXd &q) const
{
    auto J = _api._jacobian(q);

    int i = J.rows();

    J.conservativeResize(J.rows() + _eq_idx.size(), J.cols());

    J.bottomRows(_eq_idx.size()).setZero();

    for(int k : _eq_idx)
    {
        J(i, k) = 1;
        i++;
    }

    return J;
}

Eigen::VectorXd Constraint::Impl::sample() const
{
    auto atlas = getAtlas();

    if(!sampler)
    {
        sampler = atlas->allocStateSampler();
        state = std::make_shared<ompl::base::ScopedState<>>(atlas);
    }

    // cast state
    auto atlas_state = static_cast<ompl::base::AtlasStateSpaceNE::StateType*>(state->get());

    // we need to create at least a chart
    // we use ambient space sampling + projection
    if(atlas->getChartCount() == 0)
    {
        for(;;)
        {

            auto qrand = _ss->ambientRandom();

            if(!_api.project(qrand))
            {
                std::cout << "[sample] project failed \n";
                continue;
            }

            _ss->getImpl().setValue(**state, qrand);

            if(!atlas->satisfiesBounds(atlas_state))
            {
                std::cout << "[sample] satisfiesBounds failed \n";
                continue;
            }

            atlas->newChart(atlas_state);

            std::cout << "initial chart found \n";

            return qrand;
        }
    }

    // sample from current atlas
    sampler->sampleUniform(state->get());

    // create chart for sampled point if needed
    atlas->getChart(atlas_state);

    return _ss->getImpl().getValue(**state);
}

std::shared_ptr<ompl::base::AtlasStateSpaceNE> Constraint::Impl::getAtlas() const
{
    if(!_atlas)
    {
        throw std::runtime_error("could not get atlas from constraint: an unbound state");
    }

    return _atlas;
}
