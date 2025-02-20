#include <cartesio_planning/constraint.h>
#include <iostream>
#include <cartesio_planning/state_space.h>

#include "impl/constraint.hxx"
#include "impl/profiling.hxx"


using namespace XBot::Cartesian::Planning;


Constraint::Constraint(std::shared_ptr<const StateSpace> space,
           std::string options):
    Constraint(space, YAML::Load(options))
{

}


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

    for(int k = 0; k < 200; k++)
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
                // std::cout << k << ": accept alpha = " << alpha << ", cost = " << val.norm() << " < " << cost << " !! \n";
                break;
            }

            // on failure, reduce step size
            alpha *= 0.5;

        }

        q = qproj_tmp;
    }

    std::cout << "FAILED " << value(qproj_tmp).transpose() << "\n";

    return false;
}

void Constraint::setRefineTarget(const Eigen::VectorXd &q0)
{
    impl->setRefineTarget(q0);
}

bool Constraint::refine(Eigen::VectorXd &q) const
{
    auto is_valid = [this](const Eigen::VectorXd& q)
    {
        return _space->checkValid(q);
    };

    Eigen::MatrixXd J;
    Eigen::VectorXd dq, val = value(q);



    for(int iter = 0; iter < 100; iter++)
    {
        // compute GN step
        val = value(q);
        J = jacobian(q);

        auto svd = J.jacobiSvd(Eigen::ComputeFullU|Eigen::ComputeFullV);

        Eigen::MatrixXd N = svd.matrixV().rightCols(J.cols() - J.rows());

        dq = svd.solve(-val) + N*N.transpose()*_space->difference(impl->getRefineTarget(), q);

        if(dq.norm() < 1e-3)
        {
            std::cout << "dq small norm \n";
            return true;
        }

        double alpha = 1.0;

        for(;;)
        {
            std::cout << iter << ": try alpha = " << alpha << "\n";

            auto qtmp = _space->sum(q, dq*alpha);

            project(qtmp);

            if(is_valid(qtmp))
            {
                q = qtmp;
                break;
            }

            alpha *= 0.5;

            if(alpha < 1e-3)
            {
                return true;
            }
        }
    }

    return false;
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

Eigen::VectorXd Constraint::sampleGaussian(const Eigen::VectorXd &q0, double stddev) const
{
    return impl->sampleGaussian(q0, stddev);
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

    _atlas->setEpsilon(0.1);
    _atlas->setRho(2.0);
    _atlas->setExploration(0.8);

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

void Constraint::Impl::setRefineTarget(const Eigen::VectorXd &q0)
{
    _qrefinetgt = q0;
}

const Eigen::VectorXd &Constraint::Impl::getRefineTarget() const
{
    return _qrefinetgt;
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

Eigen::VectorXd Constraint::Impl::sampleGaussian(const Eigen::VectorXd &q0, double stddev) const
{
    // project mean
    auto qmean = q0;

    if(!_api.project(qmean))
    {
        throw std::runtime_error("[sampleGaussian] failed to project q0 onto constraint mainifold");
    }

    // get atlas and sampler
    auto atlas = getAtlas();

    if(!sampler)
    {
        sampler = atlas->allocStateSampler();
        state = std::make_shared<ompl::base::ScopedState<>>(atlas);
    }


    // create chart for mean value
    auto mean_state = std::make_shared<ompl::base::ScopedState<ompl::base::AtlasStateSpaceNE>>(atlas);
    _ss->getImpl().setValue(**mean_state, qmean);
    atlas->newChart((*mean_state).get());

    // sample
    sampler->sampleGaussian(state->get(), mean_state->get(), stddev);

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
