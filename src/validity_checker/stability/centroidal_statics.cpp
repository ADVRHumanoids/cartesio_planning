#include "centroidal_statics.h"
#include <boost/range/algorithm.hpp>
#include <boost/range/adaptors.hpp>

using namespace XBot::Cartesian::Planning;

CentroidalStatics::CentroidalStatics(XBot::ModelInterface::ConstPtr model, const std::vector<std::string>& contact_links,
                                     const double friction_coeff, const bool optimize_torque):
    _model(model),
    _friction_coeff(friction_coeff),
    _optimize_torque(optimize_torque),
    _is_initialized(false)
{
    Eigen::Matrix3d I; I.setIdentity();
    for(unsigned int i = 0; i < contact_links.size(); ++i)
        _contacts[contact_links[i]] = I;
    init();
}

bool CentroidalStatics::setFrictionCoeff(const double friction_coeff)
{
    if(friction_coeff < 0.)
        return false;
    _friction_coeff = friction_coeff;
    return true;
}

void CentroidalStatics::setOptimizeTorque(const bool optimize_torque)
{
    _optimize_torque = optimize_torque;
}

void CentroidalStatics::setContactLinks(const std::vector<std::string>& contact_links)
{
    _contacts.clear();

    Eigen::Matrix3d I; I.setIdentity();
    for(unsigned int i = 0; i < contact_links.size(); ++i)
        _contacts[contact_links[i]] = I;

    init();
}

void CentroidalStatics::addContatcLinks(const std::vector<std::string>& contact_links)
{
    Eigen::Matrix3d I; I.setIdentity();
    for(auto link : contact_links)
    {
        if(_contacts.find(link) == _contacts.end())
            _contacts[link] = I;
    }

    init();
}

void CentroidalStatics::removeContactLinks(const std::vector<std::string>& contact_links)
{
    for(auto link : contact_links)
    {
        if(_contacts.find(link) != _contacts.end())
            _contacts.erase(link);
    }

    init();
}

bool CentroidalStatics::setContactRotationMatrix(const std::string& contact_link,
                                                 const Eigen::Matrix3d& w_R_c)
{
    if(_contacts.find(contact_link) == _contacts.end())
        return false;

    _contacts[contact_link] = w_R_c;
     _force_optim->setContactRotationMatrix(contact_link, _contacts[contact_link]);
    return true;
}

void CentroidalStatics::init()
{
    std::vector<std::string> _contact_links;
    boost::copy(_contacts | boost::adaptors::map_keys, std::back_inserter(_contact_links));

    _force_optim = boost::make_shared<OpenSoT::utils::ForceOptimization>(std::const_pointer_cast<ModelInterface>(_model),
                                                                         _contact_links,
                                                                         _optimize_torque,
                                                                         _friction_coeff);

    for(auto link : _contact_links)
        _force_optim->setContactRotationMatrix(link, _contacts[link]);

    _is_initialized = true;
}

bool CentroidalStatics::compute()
{
    Eigen::VectorXd g;
    _model->computeGravityCompensation(g);
    return _force_optim->compute(g, _Fc, _tau);
}

bool CentroidalStatics::checkStability(const double eps, const bool init_solver)
{
    if(!_is_initialized || init_solver)
        init();


    if(compute())
    {
        double res = _force_optim->getObjective();

        if(res <= eps && res >= -res)
            return true;
    }
    return false;
}
