#include <nodes/goal_generation.h>

GoalGenerator::GoalGenerator(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                             XBot::Cartesian::Planning::ValidityCheckContext& vc_context,
                             ros::NodeHandle& nh):
    _ci(ci),
    _nh(nh),
    _vc_context(vc_context)
{
    _ik = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(_ci, _ci->getIkProblem());
    _goal_sampler = std::make_shared<XBot::Cartesian::Planning::GoalSamplerBase>(_ik);

    _ros_server = std::make_shared<XBot::Cartesian::RosServerClass>(_ci, _ci->getModel());

    _goal_sampler->setValidityCheker(
                std::bind(&XBot::Cartesian::Planning::ValidityPredicateAggregate::checkAll, &_vc_context.vc_aggregate, nullptr));

    _ik->setRosServerClass(_ros_server);

    _service_a = nh.advertiseService("goal_sampler_service", &GoalGenerator::goal_sampler_service, this);
}

void GoalGenerator::update()
{
    _ros_server->run();
}

bool GoalGenerator::sample(Eigen::VectorXd& q, double time_out)
{
    return _goal_sampler->sampleGoal(q, time_out);
}

bool GoalGenerator::goal_sampler_service(cartesio_planning::CartesioGoal::Request &req, cartesio_planning::CartesioGoal::Response &res)
{
    Eigen::VectorXd q;
    if(!sample(q, req.time)){
        res.status.val = res.status.TIMEOUT;
        res.status.msg.data = "TIMEOUT";
    }
    else
    {
        res.status.val = res.status.EXACT_SOLUTION;
        res.status.msg.data = "EXACT_SOLUTION";

        res.sampled_goal.name = _ci->getModel()->getEnabledJointNames();
        res.sampled_goal.position.resize(q.size());
        Eigen::VectorXd::Map(&res.sampled_goal.position[0], q.size()) = q;
        res.sampled_goal.header.stamp = ros::Time::now();
    }
    return true;
}
