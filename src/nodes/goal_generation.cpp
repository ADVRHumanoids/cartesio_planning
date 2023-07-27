#include <nodes/goal_generation.h>

GoalGenerator::GoalGenerator(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                             XBot::Cartesian::Planning::ValidityCheckContext& vc_context):
    _ci(ci),
    _vc_context(vc_context)
{
    _ik = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(_ci);
    _goal_sampler = std::make_shared<XBot::Cartesian::Planning::GoalSamplerBase>(_ik, vc_context);

    XBot::Cartesian::RosServerClass::Options opt;
    opt.tf_prefix = "planner/goal_sampler";
    opt.ros_namespace = "planner/goal_sampler";
    _ros_server = std::make_shared<XBot::Cartesian::RosServerClass>(_ci, opt);

    _ik->setRosServerClass(_ros_server);
}

void GoalGenerator::setValidityChecker(XBot::Cartesian::Planning::ValidityCheckContext& vc_contex)
{
    throw std::runtime_error("not impl");
}

void GoalGenerator::update()
{
    _ros_server->run();
}

bool GoalGenerator::sample(Eigen::VectorXd& q, double time_out)
{
    return _goal_sampler->sampleGoal(q, time_out);
}

bool GoalGenerator::samplePostural(Eigen::VectorXd& q, double time_out)
{
    return _goal_sampler->sampleGoalPostural(q, time_out);
}

