#include <cartesio_planning/state_space.h>
#include "impl/state_space.hxx"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace XBot::Cartesian::Planning;

StateSpace::StateSpace()
{
    impl = std::make_unique<Impl>();
}

int StateSpace::addRobotConfigurationSpace(XBot::ModelInterface::ConstPtr model)
{
    auto rss = std::make_shared<RobotConfigurationSpace>(model);
    return impl->addOmplSpace(rss, "robot_" + model->getName());
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

std::pair<Eigen::VectorXd, Eigen::VectorXd> StateSpace::getBounds() const
{
    return impl->getBounds();
}

StateSpace::~StateSpace()
{

}

StateSpace::Impl &StateSpace::getImpl()
{
    return *impl;
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

std::unique_ptr<ompl::base::State> StateSpace::Impl::createState()
{
    return std::unique_ptr<ompl::base::State>(_ss->allocState());
}

Eigen::VectorXd StateSpace::Impl::getValue(const ompl::base::State &s)
{
    const auto& cs = static_cast<const ompl::base::CompoundState&>(s);

    Eigen::VectorXd value;
    value.setZero(_ss->getDimension());

    int i = 0;


    for(auto& ss : _ss_vec)
    {
        if(auto se3 = ss->as<ompl::base::SE3StateSpace::StateType>())
        {
            value[i++] = se3->getX();
            value[i++] = se3->getY();
            value[i++] = se3->getZ();

            value[i++] = se3->rotation().x;
            value[i++] = se3->rotation().y;
            value[i++] = se3->rotation().z;
            value[i++] = se3->rotation().w;
        }
        if(auto rcs = ss->as<RobotConfigurationSpace>())
        {
            rcs->q
        }
    }
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

    return _ss_vec.size() - 1;

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

XBot::ModelInterface::ConstPtr RobotConfigurationSpace::model() const
{
    return _model;
}

unsigned int RobotConfigurationSpace::getDimension() const
{
    return _model->getNv();
}

double RobotConfigurationSpace::getMaximumExtent() const
{
    auto [qmin, qmax] = _model->getJointLimits();
    return _model->difference(qmax, qmin).norm();
}

double RobotConfigurationSpace::getMeasure() const
{
    auto [qmin, qmax] = _model->getJointLimits();
    return  _model->difference(qmax, qmin).prod();
}

void RobotConfigurationSpace::enforceBounds(ompl::base::State *state) const
{
    _model->enforceJointLimits(getQ(state));
}

bool RobotConfigurationSpace::satisfiesBounds(const ompl::base::State *state) const
{
    _model->checkJointLimits(getQ(state));
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
    return std::make_shared<StateSampler>();
}

ompl::base::State *RobotConfigurationSpace::allocState() const
{
    auto s = new StateType;
    s->q = _model->getNeutralQ();
    return s;
}

void RobotConfigurationSpace::freeState(ompl::base::State *state) const
{
    delete static_cast<StateType*>(state);
}

const Eigen::VectorXd &RobotConfigurationSpace::getQ(const ompl::base::State * s)
{
    return static_cast<const StateType*>(s)->q;
}

Eigen::VectorXd &RobotConfigurationSpace::getQ(ompl::base::State * s)
{
    return static_cast<StateType*>(s)->q;
}


void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleUniform(ompl::base::State *state)
{
    getQ(state) = _model->generateRandomQ();
}

void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
{
    Eigen::VectorXd dq(_model->getNv());

    for(int i = 0; i < dq.size(); i++)
    {
        dq[i] = rng_.uniformReal(-distance, distance);
    }

    getQ(state) = _model->sum(getQ(near), dq);

    _model->enforceJointLimits(getQ(state));
}

void XBot::Cartesian::Planning::RobotConfigurationSpace::StateSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev)
{
    Eigen::VectorXd dq(_model->getNv());

    for(int i = 0; i < dq.size(); i++)
    {
        dq[i] = rng_.gaussian(0, stdDev);
    }

    getQ(state) = _model->sum(getQ(mean), dq);

    _model->enforceJointLimits(getQ(state));
}
