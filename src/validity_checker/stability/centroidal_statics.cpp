#include "centroidal_statics.h"
#include <boost/range/algorithm.hpp>
#include <boost/range/adaptors.hpp>

using namespace XBot::Cartesian::Planning;
using namespace XBot::Cartesian;

CentroidalStatics::CentroidalStatics(XBot::ModelInterface::ConstPtr model, const std::vector<std::string>& contact_links,
                                     const double friction_coeff, const bool optimize_torque):
    _model(model),
    _friction_coeff(friction_coeff),
    _optimize_torque(optimize_torque)
{
    auto yaml_problem = createYAMLProblem(contact_links, friction_coeff, optimize_torque);

    _model_internal = ModelInterface::getModel(_model->getConfigOptions());
    _model_internal->syncFrom(*_model);
    auto ctx = std::make_shared<Context>(std::make_shared<Parameters>(1.), _model_internal);

    ProblemDescription pb(yaml_problem, ctx);

    _ci = CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    _dyn_feas = std::dynamic_pointer_cast<acceleration::DynamicFeasibility>(_ci->getTask("dynamic_feasibility"));
    for(auto link : contact_links)
        _fcs[link] = std::dynamic_pointer_cast<acceleration::FrictionCone>(_ci->getTask(link + "_fc"));
}

YAML::Node CentroidalStatics::createYAMLProblem(const std::vector<std::string>& contact_links,
                       const double friction_coeff, const bool optimize_torque)
{
    YAML::Emitter yaml;
    // SOLVER OPTIONS
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "solver_options";
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "back_end" << YAML::Value << "osqp";
    yaml << YAML::Key << "regularisation" << YAML::Value << 1e-3;
    yaml << YAML::EndMap;

    // STACK
    yaml << YAML::Key << "stack";
    yaml << YAML::Value << YAML::BeginSeq;
    yaml << YAML::BeginSeq << "dynamic_feasibility" << YAML::EndSeq;
    yaml << contact_links << YAML::EndSeq;

    // CONSTRAINTS
    yaml << YAML::Key << "constraints";
    yaml << YAML::Value << YAML::BeginSeq;
    for(auto link : contact_links)
        yaml << link + "_fc" << link + "_lims";
    yaml << YAML::EndSeq;

    // TASKS & CONSTRAINTS DEFS
    std::string libname = "libcartesio_acceleration_support.so";
    yaml << YAML::Key << "dynamic_feasibility";
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "name" << YAML::Value << "dynamic_feasibility";
    yaml << YAML::Key << "lib_name" << YAML::Value << libname;
    yaml << YAML::Key << "type" << YAML::Value << "DynamicFeasibility";
    yaml << YAML::Key << "contacts" << YAML::Value << contact_links;
    yaml << YAML::Key << "dynamics" << YAML::Value << false;
    yaml << YAML::EndMap;

    for(auto link : contact_links)
    {
        yaml << YAML::Key << link + "_fc";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "name" << YAML::Value << link + "_fc";
        yaml << YAML::Key << "lib_name" << YAML::Value << libname;
        yaml << YAML::Key << "type" << YAML::Value << "FrictionCone";
        yaml << YAML::Key << "link" << YAML::Value << link;
        yaml << YAML::Key << "friction_coeff" << YAML::Value << friction_coeff;
        yaml << YAML::EndMap;
    }

    std::vector<double> f_max(6,1000.);
    std::vector<double> f_min(6,-1000.);
    if(!optimize_torque)
        f_max[3] = f_max[4] = f_max[5] = f_min[3] = f_min[4] = f_min[5] = 0.;

    for(auto link : contact_links)
    {
        yaml << YAML::Key << link + "_lims";
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "name" << YAML::Value << link + "_lims";
        yaml << YAML::Key << "lib_name" << YAML::Value << libname;
        yaml << YAML::Key << "type" << YAML::Value << "ForceLimits";
        yaml << YAML::Key << "link" << YAML::Value << link;
        yaml << YAML::Key << "min" << YAML::Value << f_min;
        yaml << YAML::Key << "max" << YAML::Value << f_max;
        yaml << YAML::EndMap;
    }

    for(auto link : contact_links)
    {
        yaml << YAML::Key << link;
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "name" << YAML::Value << link;
        yaml << YAML::Key << "lib_name" << YAML::Value << libname;
        yaml << YAML::Key << "type" << YAML::Value << "Force";
        yaml << YAML::Key << "link" << YAML::Value << link;
        yaml << YAML::EndMap;
    }

    yaml << YAML::EndMap;
    std::cout<<yaml.c_str()<<std::endl;

    return YAML::Load(yaml.c_str());
}

bool CentroidalStatics::setFrictionCoeff(const double friction_coeff)
{
    if(friction_coeff < 0.)
        return false;
    _friction_coeff = friction_coeff;

    for(auto fc : _fcs)
        fc.second->setFrictionCoeff(_friction_coeff);

    return true;
}

//void CentroidalStatics::setOptimizeTorque(const bool optimize_torque)
//{
//    _optimize_torque = optimize_torque;

//    init();
//}

//void CentroidalStatics::setContactLinks(const std::vector<std::string>& contact_links)
//{
//    _contacts.clear();

//    Eigen::Matrix3d I; I.setIdentity();
//    for(unsigned int i = 0; i < contact_links.size(); ++i)
//        _contacts[contact_links[i]] = I;

//    init();
//}

//void CentroidalStatics::addContatcLinks(const std::vector<std::string>& contact_links)
//{
//    Eigen::Matrix3d I; I.setIdentity();
//    for(auto link : contact_links)
//    {
//        if(_contacts.find(link) == _contacts.end())
//            _contacts[link] = I;
//    }

//    init();
//}

//void CentroidalStatics::removeContactLinks(const std::vector<std::string>& contact_links)
//{
//    for(auto link : contact_links)
//    {
//        if(_contacts.find(link) != _contacts.end())
//            _contacts.erase(link);
//    }

//    init();
//}

//bool CentroidalStatics::setContactRotationMatrix(const std::string& contact_link,
//                                                 const Eigen::Matrix3d& w_R_c)
//{
//    if(_contacts.find(contact_link) == _contacts.end())
//        return false;

//    _contacts[contact_link] = w_R_c;
//     _force_optim->setContactRotationMatrix(contact_link, _contacts[contact_link]);
//    return true;
//}

//void CentroidalStatics::init()
//{
//    std::vector<std::string> _contact_links;
//    boost::copy(_contacts | boost::adaptors::map_keys, std::back_inserter(_contact_links));

//    _force_optim = boost::make_shared<OpenSoT::utils::ForceOptimization>(std::const_pointer_cast<ModelInterface>(_model),
//                                                                         _contact_links,
//                                                                         _optimize_torque,
//                                                                         _friction_coeff);

//    for(auto link : _contact_links)
//        _force_optim->setContactRotationMatrix(link, _contacts[link]);

//    _is_initialized = true;
//}

bool CentroidalStatics::compute()
{
    _model_internal->syncFrom(*_model);
    return _ci->update(0., 0.);
}

bool CentroidalStatics::checkStability(const double eps, const bool init_solver)
{
    if(compute())
    {
        Eigen::VectorXd error;
        if(!_dyn_feas->getTaskError(error))
            return false;
        double res = error.norm();
        if(res <= eps)
            return true;
    }
    return false;
}
