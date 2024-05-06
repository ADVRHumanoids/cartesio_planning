#include <cartesio_planning/state_space.h>
#include "impl/state_space.hxx"
#include "impl/profiling.hxx"

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
    return impl->addOmplSpace(rss, "robot_" + model->getName());
}

XBot::ModelInterface::Ptr StateSpace::getModel(int i) const
{
    return impl->getModel(i);
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
    return addEuclidean(qmin, qmax, id);
}

int StateSpace::getNq() const
{
    return impl->getNq();
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

const StateSpace::Impl &StateSpace::getImpl() const
{
    return *impl;
}

StateSpace::Impl &StateSpace::getImpl()
{
    return *impl;
}

StateSpace::Impl::Impl()
{
    _ss = std::make_shared<ompl::base::CompoundStateSpace>();
}

int StateSpace::Impl::addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id)
{
    auto se3 = std::make_shared<ompl::base::SE3StateSpace>();
    se3->setBounds(boundsEigenToOmpl(qmin.head<3>(), qmax.head<3>()));
    return addOmplSpace(se3, id);
}

int StateSpace::Impl::addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id)
{
    auto rn = std::make_shared<ompl::base::RealVectorStateSpace>(qmin.size());
    rn->setBounds(boundsEigenToOmpl(qmin, qmax));
    return addOmplSpace(rn, id);
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
    return _ss->allocState();
}

void StateSpace::Impl::setValue(ompl::base::State &s, Eigen::VectorXd q) const
{
    auto& cs = static_cast<ompl::base::CompoundState&>(s);

    int i = 0;
    int si = 0;

    for(auto& ss : _ss_vec)
    {
        if(auto se3 = dynamic_cast<ompl::base::SE3StateSpace::StateType*>(cs.components[si]))
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
        else if(auto rcs = dynamic_cast<RobotConfigurationSpace::StateType*>(cs.components[si]))
        {
            rcs->q = q.segment(i, ss->getDimension());
            i += ss->getDimension();
        }
        else if(auto rvs = dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(cs.components[si]))
        {
            Eigen::VectorXd::Map(rvs->values, ss->getDimension()) = q.segment(i, ss->getDimension());
            i += ss->getDimension();
        }
        else
        {
            throw std::runtime_error("invalid state type");
        }

        si++;
    }

    if(i != q.size())
    {
        throw std::runtime_error("could not set state value");
    }

}

Eigen::VectorXd StateSpace::Impl::getValue(const ompl::base::State &s) const
{
    const auto& cs = static_cast<const ompl::base::CompoundState&>(s);

    Eigen::VectorXd value;
    value.setZero(_ss->getDimension());

    int i = 0;
    int si = 0;

    for(auto& ss : _ss_vec)
    {
        if(auto se3 = dynamic_cast<ompl::base::SE3StateSpace::StateType*>(cs.components[si]))
        {
            value[i++] = se3->getX();
            value[i++] = se3->getY();
            value[i++] = se3->getZ();

            value[i++] = se3->rotation().x;
            value[i++] = se3->rotation().y;
            value[i++] = se3->rotation().z;
            value[i++] = se3->rotation().w;
        }
        else if(auto rcs = dynamic_cast<RobotConfigurationSpace::StateType*>(cs.components[si]))
        {
            value.segment(i, ss->getDimension()) = rcs->q;
            i += ss->getDimension();
        }
        else if(auto rvs = dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(cs.components[si]))
        {
            value.segment(i, ss->getDimension()) = Eigen::VectorXd::Map(rvs->values, ss->getDimension());
            i += ss->getDimension();
        }
        else
        {
            throw std::runtime_error("invalid state type");
        }

        si++;
    }

    if(i != value.size())
    {
        throw std::runtime_error("could not get state value");
    }

    return value;
}

int StateSpace::Impl::addOmplSpace(ompl::base::StateSpacePtr ss, std::string id)
{
    _ss_vec.push_back(ss);

    if(!id.empty())
    {
        ss->setName(id);

        if(_ss_map.count(id))
        {
            throw std::invalid_argument("multiple definitions of id '" + id + "'");
        }

        _ss_map[id] = ss;
    }

    _ss->addSubspace(ss, 1.0);

    if(_q_index.empty())
    {
        _q_index = {0};
    }
    else
    {
        _q_index.push_back(_q_index.back() + ss->getDimension());
    }

    return _ss_vec.size() - 1;

}

ompl::base::StateSpacePtr StateSpace::Impl::getStateSpace() const
{
    return _ss;
}

int StateSpace::Impl::getNq() const
{
    return _ss->getDimension();
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

void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleUniform(ompl::base::State *state)
{
    TIKTOK(sample_q);

    getQ(state) = _model->generateRandomQ();

    if(_opt.sample_collision_free)
    {
        _opt.collision_model->computeCollisionFree(getQ(state), _opt.compute_coll_free_opt);
    }

}

void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
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

void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev)
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



void XBot::Cartesian::Planning::RobotConfigurationSpace::printState(const ompl::base::State *state, std::ostream &out) const
{
    out << getQ(state).transpose().format(2);
}
