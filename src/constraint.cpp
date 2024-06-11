#include <cartesio_planning/constraint.h>
#include <iostream>
#include <cartesio_planning/state_space.h>

#include "impl/constraint.hxx"
#include "impl/profiling.hxx"


using namespace XBot::Cartesian::Planning;


Constraint::Constraint(YAML::Node options)
{
    impl = std::make_unique<Impl>(*this, options);
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

bool Constraint::project(Eigen::VectorXd &q) const
{
    TIKTOK(constr_project);

    if(!_space)
    {
        throw std::runtime_error("cannot call project() on unbound constraint");
    }

    Eigen::VectorXd val = value(q);
    Eigen::VectorXd qproj_tmp;
    Eigen::VectorXd dq;
    Eigen::MatrixXd J;

    for(int k = 0; k < 100; k++)
    {
        J = jacobian(q);

        dq = J.jacobiSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(-val);

        double alpha = 1.0;

        double cost = val.norm();

        if(cost < 1e-4)
        {
            return true;
        }

        for(int i = 0; i < 10; i++)
        {
            qproj_tmp = _space->sum(q, alpha*dq);

            val = value(qproj_tmp);

            if(val.norm() < cost)
            {
                break;
            }

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
            return false;
        }

    }

    return true;
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
    getAtlas()->clear();
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
                continue;
            }

            _ss->getImpl().setValue(**state, qrand);

            atlas->newChart(atlas_state);

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
