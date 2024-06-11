#include <cartesio_planning/state_space.h>
#include "impl/state_space.hxx"
#include "impl/profiling.hxx"
#include "impl/parse_utils.hxx"
#include "impl/constraint.hxx"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace XBot::Cartesian::Planning;


StateSpace::StateSpace()
{
    impl = std::make_unique<Impl>();
}

int StateSpace::addRobotConfigurationSpace(XBot::ModelInterface::Ptr model,
                                           RobotConfigurationSpaceOptions opt)
{
    auto rss = std::make_shared<RobotConfigurationSpace>(model, opt);
    return impl->addOmplSpace(rss, "robot_" + model->getName(), Type::ROBOT_CONFIGURATION);
}

XBot::ModelInterface::Ptr StateSpace::getModel(int i) const
{
    return impl->getModel(i);
}

Eigen::VectorXd StateSpace::sum(const Eigen::VectorXd &q1, const Eigen::VectorXd &v) const
{
    return impl->ambientSum(q1, v);
}

Eigen::VectorXd StateSpace::interpolate(const Eigen::VectorXd &q1,
                                        const Eigen::VectorXd &q2,
                                        double tau) const
{
    return impl->interpolate(q1, q2, tau);
}

Eigen::VectorXd StateSpace::difference(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2) const
{
    return impl->ambientDiff(q1, q2);
}

bool StateSpace::addStateValidityChecker(std::shared_ptr<const StateValidityChecker> svc)
{
    return impl->addStateValidityChecker(svc);
}

bool StateSpace::checkValid(const Eigen::VectorXd &q,
                            std::vector<std::string> *failed_checks,
                            std::ostream &report_os) const
{
    return impl->checkValid(q, bool(failed_checks), report_os, failed_checks);
}

// StateSpace::StateSpace(XBotInterface &model)
// {
//     for(auto j : model.getJoints())
//     {
//         switch(j->getType())
//         {
//         case urdf::Joint::REVOLUTE:
//         case urdf::Joint::PRISMATIC:
//             addEuclidean(j->getJointLimits().first,
//                          j->getJointLimits().second,
//                          j->getName());
//             break;
//         case urdf::Joint::FLOATING:
//             addSE3(j->getJointLimits().first,
//                    j->getJointLimits().second,
//                    j->getName());
//             break;
//         case urdf::Joint::CONTINUOUS:
//             addSO2(j->getJointLimits().first[0],
//                    j->getJointLimits().second[0],
//                    j->getName());
//             break;
//         default:
//             throw std::runtime_error("invalid joint type for " + j->getName());
//             break;
//         }
//     }
// }


int StateSpace::addSE3(Eigen::Vector6d qmin,
                       Eigen::Vector6d qmax,
                       std::string id)
{
    return impl->addSE3(qmin, qmax, id);
}

int StateSpace::addEuclidean(Eigen::VectorXd qmin,
                             Eigen::VectorXd qmax,
                             std::string id)
{
    return impl->addEuclidean(qmin, qmax, id);
}

void StateSpace::setConstraint(Constraint::Ptr c)
{
    return impl->setConstraint(c);
}

Eigen::VectorXd StateSpace::random() const
{
    return impl->random();
}

Eigen::VectorXd StateSpace::ambientRandom() const
{
    return impl->ambientRandom();
}

int StateSpace::getNq() const
{
    return impl->getNq();
}

int StateSpace::getNv() const
{
    return impl->getNv();
}

int StateSpace::getNq(int i) const
{
    return impl->getNq(i);
}

int StateSpace::getQIndex(int i) const
{
    return impl->getQIndex(i);
}

StateSpace::~StateSpace()
{

}

StateSpace::Impl &StateSpace::getImpl() const
{
    return *impl;
}

StateSpace::Impl &StateSpace::getImpl()
{
    return *impl;
}

StateSpace::Impl::Impl():
    _ss_nv(0)
{
    _ss_comp = std::make_shared<ompl::base::CompoundStateSpace>();
}

void StateSpace::Impl::setOptions(YAML::Node options)
{
    _options = options;
}

int StateSpace::Impl::addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id)
{
    auto se3 = std::make_shared<ompl::base::SE3StateSpace>();
    se3->setBounds(boundsEigenToOmpl(qmin.head<3>(), qmax.head<3>()));
    return addOmplSpace(se3, id, Type::SE3);
}

int StateSpace::Impl::addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id)
{
    auto rn = std::make_shared<ompl::base::RealVectorStateSpace>(qmin.size());
    rn->setBounds(boundsEigenToOmpl(qmin, qmax));
    return addOmplSpace(rn, id, Type::EUCLIDEAN);
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> StateSpace::Impl::getBounds() const
{
    Eigen::VectorXd qmin, qmax;
    qmin.setConstant(_ss->getDimension(), -INFINITY);
    qmax.setConstant(_ss->getDimension(), INFINITY);

    int i = 0;

    for(auto ss : _ss_vec)
    {
        if(auto se3 = std::dynamic_pointer_cast<ompl::base::SE3StateSpace>(ss))
        {
            qmin.segment<3>(i) = vectorToEigen(se3->getBounds().low);
            qmax.segment<3>(i) = vectorToEigen(se3->getBounds().high);
        }
        else if(auto rn = std::dynamic_pointer_cast<ompl::base::RealVectorStateSpace>(ss))
        {
            qmin.segment(i, rn->getDimension()) = vectorToEigen(rn->getBounds().low);
            qmax.segment(i, rn->getDimension()) = vectorToEigen(rn->getBounds().high);
        }
        else if(auto rcs = std::dynamic_pointer_cast<RobotConfigurationSpace>(ss))
        {
            qmin.segment(i, rn->getDimension()) = rcs->model()->getJointLimits().first;
            qmax.segment(i, rn->getDimension()) = rcs->model()->getJointLimits().second;
        }
        else
        {
            throw std::runtime_error("unknown state space type");
        }

        i += ss->getDimension();
        continue;
    }

    return {qmin, qmax};
}

ompl::base::State * StateSpace::Impl::createState()const
{
    return getStateSpace()->allocState();
}

void StateSpace::Impl::setValue(ompl::base::State &s, Eigen::VectorXd q) const
{
    //
    auto set_value_impl = [](ompl::base::State* s,
                             const Eigen::VectorXd& q,
                             int size,
                             int& i)
    {
        if(auto ws = dynamic_cast<ompl::base::WrapperStateSpace::StateType*>(s))
        {
            s = ws->getState();
        }

        if(auto se3 = dynamic_cast<ompl::base::SE3StateSpace::StateType*>(s))
        {
            // set r^3
            Eigen::Vector3d::Map(se3->as<ompl::base::RealVectorStateSpace::StateType>(0)->values)
                = q.segment<3>(i);

            i += 3;

            // set so3
            auto so3 = se3->as<ompl::base::SO3StateSpace::StateType>(1);

            so3->x = q[i++];
            so3->y = q[i++];
            so3->z = q[i++];
        }
        else if(auto rcs = dynamic_cast<RobotConfigurationSpace::StateType*>(s))
        {
            rcs->q = q.segment(i, size);
            i += size;
        }
        else if(auto rvs = dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(s))
        {
            Eigen::VectorXd::Map(rvs->values, size) = q.segment(i, size);
            i += size;
        }
        else
        {
            throw std::runtime_error("invalid state type");
        }
    };

    // handle simple state case
    int i = 0;

    if(_ss != _ss_comp)
    {
        set_value_impl(&s, q, _ss->getDimension(), i);
    }
    else
    {
        auto& cs = static_cast<ompl::base::CompoundState&>(s);

        int si = 0;

        for(auto& ss : _ss_vec)
        {
            set_value_impl(cs.components[si], q, ss->getDimension(), i);

            si++;
        }
    }

    if(i != q.size())
    {
        throw std::runtime_error("could not set state value");
    }

}

Eigen::VectorXd StateSpace::Impl::getValue(const ompl::base::State &s) const
{
    //
    auto get_value_impl = [](const ompl::base::State* s,
                             Eigen::VectorXd& value,
                             int size,
                             int& i)
    {
        if(auto ws = dynamic_cast<const ompl::base::WrapperStateSpace::StateType*>(s))
        {
            s = ws->getState();
        }

        if(auto se3 = dynamic_cast<const ompl::base::SE3StateSpace::StateType*>(s))
        {
            value[i++] = se3->getX();
            value[i++] = se3->getY();
            value[i++] = se3->getZ();

            value[i++] = se3->rotation().x;
            value[i++] = se3->rotation().y;
            value[i++] = se3->rotation().z;
            value[i++] = se3->rotation().w;
        }
        else if(auto rcs = dynamic_cast<const RobotConfigurationSpace::StateType*>(s))
        {
            value.segment(i, size) = rcs->q;
            i += size;
        }
        else if(auto rvs = dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(s))
        {
            value.segment(i, size) = Eigen::VectorXd::Map(rvs->values, size);
            i += size;
        }
        else
        {
            throw std::runtime_error("invalid state type");
        }
    };

    Eigen::VectorXd value;
    value.setZero(_ss->getDimension());

    int i = 0;

    if(_ss != _ss_comp)
    {
        get_value_impl(&s, value, _ss->getDimension(), i);
    }
    else
    {
        const auto& cs = static_cast<const ompl::base::CompoundState&>(s);

        int si = 0;

        for(auto& ss : _ss_vec)
        {
            get_value_impl(cs.components[si], value, ss->getDimension(), i);

            si++;
        }
    }

    if(i != value.size())
    {
        throw std::runtime_error("could not get state value");
    }

    return value;
}

int StateSpace::Impl::addOmplSpace(ompl::base::StateSpacePtr ss,
                                   std::string id,
                                   Type type)
{
    // add to state space vector
    _ss_vec.push_back(ss);

    // set id
    if(!id.empty())
    {
        ss->setName(id);

        if(_ss_map.count(id))
        {
            throw std::invalid_argument("multiple definitions of id '" + id + "'");
        }

        _ss_map[id] = ss;
    }

    // add to compound ss
    _ss_comp->addSubspace(ss, 1.0);

    // save q index
    if(_q_index.empty())
    {
        _q_index = {0};
    }
    else
    {
        _q_index.push_back(_q_index.back() + ss->getDimension());
    }

    // save non-euclidean space information
    BinaryVectorOp fsum, fdiff;
    int nv = 0;

    if(type == Type::EUCLIDEAN)
    {
        nv = ss->getDimension();

        fsum = [](const Eigen::VectorXd& q, const Eigen::VectorXd& v)
        {
            return q + v;
        };

        fdiff = [](const Eigen::VectorXd& q1, const Eigen::VectorXd& q2)
        {
            return q1 - q2;
        };
    }
    else if(type == Type::ROBOT_CONFIGURATION)
    {
        auto model = getModel(_ss_vec.size() - 1);
        nv = model->getNv();

        fsum = [model](const Eigen::VectorXd& q, const Eigen::VectorXd& v)
        {
            return model->sum(q, v);
        };

        fdiff = [model](const Eigen::VectorXd& q1, const Eigen::VectorXd& q2)
        {
            return model->difference(q1, q2);
        };
    }
    else if(type == Type::SO2)
    {
        nv = 1;
        throw std::runtime_error("unsupported");
    }
    else if(type == Type::SE2)
    {
        nv = 3;
        throw std::runtime_error("unsupported");
    }
    else if(type == Type::SE3)
    {
        nv = 6;
        throw std::runtime_error("unsupported");
    }
    else
    {
        throw std::runtime_error("invalid state space type");
    }

    _ss_nv += nv;

    _nv.push_back(nv);

    if(_v_index.empty())
    {
        _v_index = {0};
    }
    else
    {
        _v_index.push_back(_v_index.back() + nv);
    }

    _f_sum.push_back(fsum);
    _f_diff.push_back(fdiff);

    // planning state space
    if(_ss_vec.size() == 1)
    {
        _ss = ss;
    }
    else
    {
        _ss = _ss_comp;
    }

    return _ss_vec.size() - 1;

}

ompl::base::StateSpacePtr StateSpace::Impl::getStateSpace() const
{
    return _constr ? _constr->getImpl().getAtlas() : _ss;
}

ompl::base::StateSpacePtr StateSpace::Impl::getAmbientStateSpace() const
{
    return _ss;
}

int StateSpace::Impl::getNq() const
{
    return _ss->getDimension();
}

int StateSpace::Impl::getNv() const
{
    return _ss_nv;
}

int StateSpace::Impl::getNq(int i) const
{
    return _ss_vec.at(i)->getDimension();
}

int StateSpace::Impl::getQIndex(int i) const
{
    return _q_index.at(i);
}

XBot::ModelInterface::Ptr StateSpace::Impl::getModel(int i) const
{
    auto rcs = std::dynamic_pointer_cast<RobotConfigurationSpace>(_ss_vec.at(i));

    if(!rcs)
    {
        return nullptr;
    }

    return rcs->model();
}

void StateSpace::Impl::updateModelState(const Eigen::VectorXd& q) const
{
    TIKTOK(update_model_state);

    for(int i = 0; i < _ss_vec.size(); i++)
    {
        if(auto model = getModel(i))
        {
            model->setJointPosition(q.segment(getQIndex(i), getNq(i)));
            model->update();
        }
    }
}

void StateSpace::Impl::setConstraint(Constraint::Ptr c)
{
    _constr = c;
}

Constraint::Ptr StateSpace::Impl::getConstraint() const
{
    return _constr;
}

Eigen::VectorXd StateSpace::Impl::ambientSum(const Eigen::VectorXd &q,
                                             const Eigen::VectorXd &v)
{
    Eigen::VectorXd ret(_ss->getDimension());

    for(int i = 0; i < _ss_vec.size(); i++)
    {
        int iq = getQIndex(i);
        int nq = getNq(i);
        int iv = _v_index[i];
        int nv = _nv[i];

        ret.segment(iq, nq) = _f_sum[i](q.segment(iq, nq), v.segment(iv, nv));
    }


    return ret;
}

Eigen::VectorXd StateSpace::Impl::ambientDiff(const Eigen::VectorXd &q1,
                                              const Eigen::VectorXd &q0)
{
    Eigen::VectorXd ret(_ss_nv);

    for(int i = 0; i < _ss_vec.size(); i++)
    {
        int iq = getQIndex(i);
        int nq = getNq(i);
        int iv = _v_index[i];
        int nv = _nv[i];

        ret.segment(iv, nv) = _f_diff[i](q1.segment(iq, nq), q0.segment(iq, nq));
    }

    return ret;
}

Eigen::VectorXd StateSpace::Impl::interpolate(const Eigen::VectorXd &q1,
                                              const Eigen::VectorXd &q2,
                                              double tau)
{
    auto ss = getStateSpace();

    ompl::base::ScopedState<> s1(ss), s2(ss), si(ss);

    setValue(*s1, q1);
    setValue(*s2, q2);

    ss->interpolate(s1.get(), s2.get(), tau, si.get());

    return getValue(*si);
}

void StateSpace::Impl::setSpaceInformation(ompl::base::SpaceInformation *si)
{
    if(_constr)
    {
        _constr->getImpl().getAtlas()->setSpaceInformation(si);
    }
}

Eigen::VectorXd StateSpace::Impl::ambientRandom() const
{
    ompl::base::ScopedState<> s(_ss);
    _ss->allocStateSampler()->sampleUniform(s.get());
    return getValue(*s);
}

Eigen::VectorXd StateSpace::Impl::random() const
{
    if(!_constr) return ambientRandom();

    auto ss = _constr->getImpl().getAtlas();
    ompl::base::ScopedState<> s(ss);
    ss->allocStateSampler()->sampleUniform(s.get());
    return getValue(*s);
}

bool StateSpace::Impl::addStateValidityChecker(StateValidityChecker::ConstPtr svc)
{
    _svc_map[svc->getName()] = svc;
    return true;
}

bool StateSpace::Impl::isStateValid(const ompl::base::State &state,
                                 bool force_verbose,
                                 std::ostream& os,
                                 std::vector<std::string> * failed_checks) const
{

    Eigen::VectorXd q = getValue(state);

    return checkValid(q, force_verbose, os, failed_checks);

}

bool StateSpace::Impl::checkValid(const Eigen::VectorXd &q,
                                  bool force_verbose,
                                  std::ostream &os,
                                  std::vector<std::string> *failed_checks) const
{
    TIKTOK(state_validity_check);

    updateModelState(q);

    std::optional<Eigen::VectorXd> qnear;

    for(const auto& [cname, c] : _svc_map)
    {
        qnear.reset();

        if(!c->checkValid(q, qnear))
        {
            if(force_verbose)
            {
                os << "validity check '" << cname << "' failed: ";
                c->printInvalidStateInformation(os);
                os << "\n";
            }

            if(failed_checks)
            {
                failed_checks->push_back(cname);
            }

            return false;
        }
    }

    TIKTOK(state_valid);

    return true;

}

namespace XBot::Cartesian::Planning
{

ompl::base::RealVectorBounds boundsEigenToOmpl(Eigen::VectorXd qmin, Eigen::VectorXd qmax)
{
    if(qmin.size() != qmax.size())
    {
        throw std::invalid_argument("bound size mismatch");
    }

    ompl::base::RealVectorBounds ompl_bounds(qmin.size());

    Eigen::VectorXd::Map(ompl_bounds.low.data(),
                         qmin.size()) = qmin;

    Eigen::VectorXd::Map(ompl_bounds.high.data(),
                         qmin.size()) = qmax;

    return ompl_bounds;
}

Eigen::VectorXd vectorToEigen(std::vector<double> v)
{
    return Eigen::VectorXd::Map(v.data(), v.size());
}


}

StateSpace::RobotConfigurationSpaceOptions::RobotConfigurationSpaceOptions()
{
    sample_collision_free = false;
}


RobotConfigurationSpace::RobotConfigurationSpace(ModelInterface::Ptr model,
                                                 Planning::StateSpace::RobotConfigurationSpaceOptions opt):
    _model(model), _opt(opt)
{

}

XBot::ModelInterface::Ptr RobotConfigurationSpace::model() const
{
    return _model;
}

unsigned int RobotConfigurationSpace::getDimension() const
{
    return _model->getNq();
}

double RobotConfigurationSpace::getMaximumExtent() const
{
    auto [qmin, qmax] = _model->getJointLimits();
    return (qmax - qmin).norm();
}

double RobotConfigurationSpace::getMeasure() const
{
    auto [qmin, qmax] = _model->getJointLimits();
    return  (qmax - qmin).prod();
}

void RobotConfigurationSpace::enforceBounds(ompl::base::State *state) const
{
    _model->enforceJointLimits(getQ(state));
}

bool RobotConfigurationSpace::satisfiesBounds(const ompl::base::State *state) const
{
    bool ok = _model->checkJointLimits(getQ(state));
    return ok;
}

void RobotConfigurationSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const
{
    getQ(destination) = getQ(source);
}

double RobotConfigurationSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    return _model->difference(getQ(state1), getQ(state2)).norm();
}

bool RobotConfigurationSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    return _model->difference(getQ(state1), getQ(state2)).lpNorm<Eigen::Infinity>() < 1e-6;
}

void RobotConfigurationSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const
{
    auto diff = _model->difference(getQ(to), getQ(from));
    getQ(state) = _model->sum(getQ(from), t * diff);
}

ompl::base::StateSamplerPtr RobotConfigurationSpace::allocDefaultStateSampler() const
{
    return std::make_shared<StateSampler>(this, _model, _opt);
}

ompl::base::State *RobotConfigurationSpace::allocState() const
{
    auto s = new StateType;
    s->q = _model->getNeutralQ();
    return s;
}

void RobotConfigurationSpace::freeState(ompl::base::State *state) const
{
    // delete static_cast<StateType*>(state);
}

const Eigen::VectorXd &RobotConfigurationSpace::getQ(const ompl::base::State * s)
{
    return static_cast<const StateType*>(s)->q;
}

Eigen::VectorXd &RobotConfigurationSpace::getQ(ompl::base::State * s)
{
    return static_cast<StateType*>(s)->q;
}


RobotConfigurationSpace::StateSampler::StateSampler(const StateSpace *space,
                                                    ModelInterface::ConstPtr model,
                                                    Planning::StateSpace::RobotConfigurationSpaceOptions opt):
    ompl::base::StateSampler(space),
    _model(model),
    _opt(opt)
{

}

void RobotConfigurationSpace::StateSampler::sampleUniform(ompl::base::State *state)
{
    TIKTOK(sample_q);

    getQ(state) = _model->generateRandomQ();

    if(_opt.sample_collision_free)
    {
        _opt.collision_model->computeCollisionFree(getQ(state), _opt.compute_coll_free_opt);
    }

}

void RobotConfigurationSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
{
    TIKTOK(sample_q);

    Eigen::VectorXd dq(_model->getNv());

    for(int i = 0; i < dq.size(); i++)
    {
        dq[i] = rng_.uniformReal(-distance, distance);
    }

    getQ(state) = _model->sum(getQ(near), dq);

    _model->enforceJointLimits(getQ(state));

    if(_opt.sample_collision_free)
    {
        _opt.collision_model->computeCollisionFree(getQ(state), _opt.compute_coll_free_opt);
    }
}

void RobotConfigurationSpace::StateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev)
{
    TIKTOK(sample_q);

    Eigen::VectorXd dq(_model->getNv());

    for(int i = 0; i < dq.size(); i++)
    {
        dq[i] = rng_.gaussian(0, stdDev);
    }

    getQ(state) = _model->sum(getQ(mean), dq);

    _model->enforceJointLimits(getQ(state));

    if(_opt.sample_collision_free)
    {
        _opt.collision_model->computeCollisionFree(getQ(state), _opt.compute_coll_free_opt);
    }
}



void RobotConfigurationSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
    out << getQ(state).transpose().format(2);
}

ConstraintWrapper::ConstraintWrapper(int ambient_dim,
                                     ::Constraint::Ptr constraint):
    ompl::base::Constraint(ambient_dim,
                           constraint->constraintSize()),
    _c(constraint)
{

}

void ConstraintWrapper::function(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    TIKTOK(constr_eval);

    auto val = _c->value(x);

    if(val.size() != out.size())
    {
        std::stringstream ss;

        ss << "value size mismatch: " <<
            val.size() << " != " << out.size();

        throw std::invalid_argument(ss.str());
    }

    out = std::move(val);
}

void ConstraintWrapper::jacobian(
    const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
{
    TIKTOK(constr_jac);

    auto J = _c->jacobian(x);

    if(out.rows() != J.rows() ||
        out.cols() != J.cols())
    {
        std::stringstream ss;

        ss << "jacobian size mismatch: " <<
            out.rows() << " != " << J.rows() <<
            " || " << out.cols() << " != " << J.cols();

        throw std::invalid_argument(ss.str());
    }

    out = std::move(J);
}

double *RobotConfigurationSpace::getValueAddressAtIndex(ompl::base::State *state,
                                                        unsigned int index) const
{
    auto& q = getQ(state);
    return index < q.size() ? &q[index] : nullptr;
}
