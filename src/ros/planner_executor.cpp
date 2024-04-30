#include "planner_executor.h"

#include <xbot2_interface/ros/config_from_param.hpp>

#include <sensor_msgs/JointState.h>

using namespace XBot::Cartesian::Planning;



PlannerExecutor::PlannerExecutor():
    _npr("~")
{
    auto cfg = Utils::ConfigOptionsFromParamServer();

    _robot = RobotInterface::getRobot(cfg);

    _model = ModelInterface::getModel(cfg);

    auto state_space = std::make_shared<StateSpace>();

    StateSpace::RobotConfigurationSpaceOptions ss_opt;

    state_space->addRobotConfigurationSpace(_model, ss_opt);

    _planner = std::make_shared<Planner>(state_space, YAML::Node());

    _ps = std::make_shared<PlanningSceneWrapper>(_model);

    _planner->addStateValidityChecker(
        std::make_shared<PlanningSceneChecker>(_ps, state_space)
        );

    _q_start = _q_goal = _model->getRobotState("home");

    _viz_start = std::make_shared<RobotViz>(_model, "markers/start", _npr, Eigen::Vector4d(0, 0, 1, 0.5));

    _viz_goal = std::make_shared<RobotViz>(_model, "markers/goal", _npr, Eigen::Vector4d(0, 1, 0, 0.5));

    _viz_solution = std::make_shared<RobotViz>(_model, "markers/solution", _npr, Eigen::Vector4d(0, 1, 1, 1));

    publishMarkerStart();

    publishMarkerGoal();

    _start_sub = _npr.subscribe<sensor_msgs::JointState>(
        "start", 1,
        [this](const sensor_msgs::JointStateConstPtr& msg)
        {
            _model->setJointPosition(_q_start);

            Eigen::VectorXd q = _model->positionToMinimal(_q_start);

            for(int i = 0; i < msg->name.size(); i++)
            {
                q[_model->getVIndexFromVName(msg->name[i])] = msg->position[i];
            }

            _model->minimalToPosition(q, _q_start);

            publishMarkerStart();
        });

    _goal_sub = _npr.subscribe<sensor_msgs::JointState>(
        "goal", 1,
        [this](const sensor_msgs::JointStateConstPtr& msg)
        {
            _model->setJointPosition(_q_goal);

            Eigen::VectorXd q = _model->positionToMinimal(_q_goal);

            for(int i = 0; i < msg->name.size(); i++)
            {
                q[_model->getVIndexFromVName(msg->name[i])] = msg->position[i];
            }

            _model->minimalToPosition(q, _q_goal);

            publishMarkerGoal();
        });


}

void PlannerExecutor::publishMarkerStart()
{
    _planner->checkValid(_q_start);
    _viz_start->publishMarkers(ros::Time::now(), _ps->getCollidingLinks());
}

void PlannerExecutor::publishMarkerGoal()
{
    _planner->checkValid(_q_goal);
    _viz_start->publishMarkers(ros::Time::now(), _ps->getCollidingLinks());
}

