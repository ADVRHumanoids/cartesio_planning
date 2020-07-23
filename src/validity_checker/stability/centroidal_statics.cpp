#include "centroidal_statics.h"
#include <boost/range/algorithm.hpp>
#include <boost/range/adaptors.hpp>

using namespace XBot::Cartesian::Planning;
using namespace XBot::Cartesian;

CentroidalStatics::CentroidalStatics(XBot::ModelInterface::ConstPtr model, const std::vector<std::string>& contact_links,
                                     const double friction_coeff,
                                     const bool optimize_torque,
                                     const Eigen::Vector2d& xlims_cop,
                                     const Eigen::Vector2d& ylims_cop):
    _model(model),
    _friction_coeff(friction_coeff),
    _optimize_torque(optimize_torque),
    _contact_links(contact_links),
    _x_lims_cop(xlims_cop),
    _y_lims_cop(ylims_cop)
{
    init();
}

void CentroidalStatics::init()
{
    auto yaml_problem = createYAMLProblem(_contact_links, _friction_coeff, _optimize_torque);

    _model_internal = ModelInterface::getModel(_model->getConfigOptions());
    _model_internal->syncFrom(*_model);
    auto ctx = std::make_shared<Context>(std::make_shared<Parameters>(1.), _model_internal);

    ProblemDescription pb(yaml_problem, ctx);

    _ci = CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    _dyn_feas = std::dynamic_pointer_cast<acceleration::DynamicFeasibility>(_ci->getTask("dynamic_feasibility"));
    if(_fcs.size() > 0)
        _fcs.clear();
    for(auto link : _contact_links)
        _fcs[link] = std::dynamic_pointer_cast<acceleration::FrictionCone>(_ci->getTask(link + "_fc"));
    if(_fs.size() > 0)
        _fs.clear();
    for(auto link : _contact_links)
        _fs[link] = std::dynamic_pointer_cast<acceleration::ForceTask>(_ci->getTask(link));
}

YAML::Node CentroidalStatics::createYAMLProblem(const std::vector<std::string>& contact_links,
                       const double friction_coeff, const bool optimize_torque)
{
    YAML::Emitter yaml;
    // SOLVER OPTIONS
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "solver_options";
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "back_end" << YAML::Value << "qpoases";
    yaml << YAML::Key << "regularisation" << YAML::Value << 1e-4;
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
    if(_optimize_torque)
    {
        for(auto link : contact_links)
            yaml << link + "_cop";
    }
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

    if(_optimize_torque)
    {
        std::vector<double> x,y;
        x.push_back(_x_lims_cop[0]); x.push_back(_x_lims_cop[1]);
        y.push_back(_y_lims_cop[0]); y.push_back(_y_lims_cop[1]);
        for(auto link : contact_links)
        {
            yaml << YAML::Key << link + "_cop";
            yaml << YAML::BeginMap;
            yaml << YAML::Key << "name" << YAML::Value << link + "_cop";
            yaml << YAML::Key << "lib_name" << YAML::Value << libname;
            yaml << YAML::Key << "type" << YAML::Value << "CoP";
            yaml << YAML::Key << "link" << YAML::Value << link;
            yaml << YAML::Key << "x_limits" << x;
            yaml << YAML::Key << "y_limits" << x;
            yaml << YAML::EndMap;
        }
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

void CentroidalStatics::setOptimizeTorque(const bool optimize_torque)
{
    _optimize_torque = optimize_torque;

    init();
}

void CentroidalStatics::setContactLinks(const std::vector<std::string>& contact_links)
{
    _contact_links.clear();
    _contact_links = contact_links;

    init();
}

void CentroidalStatics::addContactLinks(const std::vector<std::string>& contact_links)
{
    std::map<std::string, Eigen::Matrix3d> rotations;
    for(auto fc : _fcs)
        rotations[fc.first] = fc.second->getContactFrame();

    for(auto contact_link : contact_links)
        _contact_links.push_back(contact_link);

    init();

    for(auto R : rotations)
        setContactRotationMatrix(R.first, R.second);
}

void CentroidalStatics::removeContactLinks(const std::vector<std::string>& contact_links)
{
    std::map<std::string, Eigen::Matrix3d> rotations;
    for(auto fc : _fcs)
        rotations[fc.first] = fc.second->getContactFrame();

    for(auto link : contact_links)
    {
        std::vector<std::string>::iterator it = std::find(_contact_links.begin(), _contact_links.end(), link);
        if(it != _contact_links.end())
        {
            _contact_links.erase(it);
            rotations.erase(link);
        }
    }

    init();

    for(auto R : rotations)
        setContactRotationMatrix(R.first, R.second);
}

bool CentroidalStatics::setContactRotationMatrix(const std::string& contact_link,
                                                 const Eigen::Matrix3d& w_R_c)
{
    if(_fcs.find(contact_link) == _fcs.end())
        return false;
    _fcs[contact_link]->setContactRotationMatrix(w_R_c);
    return true;
}

bool CentroidalStatics::compute()
{
    _model_internal->syncFrom(*_model);
    return _ci->update(0., 0.);
}

bool CentroidalStatics::checkStability(const double eps)
{
    if(compute())
    {
        Eigen::VectorXd error;
        if(!_dyn_feas->getTaskError(error))
            return false;
        double res = error.norm();
        //std::cout<<"error.norm(): "<<error.norm()<<std::endl;
        if(res <= eps)
            return true;
    }
    return false;
}

const std::map<std::string, Eigen::Vector6d>& CentroidalStatics::getForces()
{
    _Fc.clear();

    for(auto f : _fs)
        _Fc[f.first] = f.second->getForceValue();
    return _Fc;
}
