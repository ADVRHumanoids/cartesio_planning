#ifndef _GOAL_GENERATION_H_
#define _GOAL_GENERATION_H_

#include <XBotInterface/ModelInterface.h>
#include <ros/node_handle.h>
#include <validity_checker/validity_checker_context.h>
#include <goal/goal_sampler.h>
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesio_planning/CartesioGoal.h>

class GoalGenerator
{
public:
    typedef std::shared_ptr<GoalGenerator> Ptr;

    GoalGenerator(XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                  XBot::Cartesian::Planning::ValidityCheckContext& vc_context);

    void update();

    bool sample(Eigen::VectorXd& q, double time_out);

    bool setErrorTolerance(const double error_tolerance)
    {
        if(_ik)
        {
            _ik->setErrorTolerance(error_tolerance);
            return true;
        }
        return false;
    }

    bool setMaxIterations(const int max_iter)
    {
        if(_ik)
        {
            _ik->setMaxIterations(max_iter);
            return true;
        }
        return false;
    }

private:
    XBot::Cartesian::Planning::ValidityCheckContext& _vc_context;

    XBot::Cartesian::Planning::GoalSamplerBase::Ptr _goal_sampler;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::RosServerClass::Ptr _ros_server;
    XBot::Cartesian::Planning::PositionCartesianSolver::Ptr _ik;






};

#endif
