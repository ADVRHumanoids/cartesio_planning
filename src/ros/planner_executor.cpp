#include "planner_executor.h"

#include <functional>

#include <xbot2_interface/ros/config_from_param.hpp>

#include <sensor_msgs/JointState.h>

using namespace XBot::Cartesian::Planning;



PlannerExecutor::PlannerExecutor():
    _npr("~")
{
    auto cfg = Utils::ConfigOptionsFromParamServer();

    try
    {
        _robot = RobotInterface::getRobot(cfg);
    }
    catch(std::runtime_error&)
    {

    }

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
            _q_start = jointStateToQ(*msg, _q_start);

            publishMarkerStart();
        });

    _goal_sub = _npr.subscribe<sensor_msgs::JointState>(
        "goal", 1,
        [this](const sensor_msgs::JointStateConstPtr& msg)
        {
            _q_goal = jointStateToQ(*msg, _q_goal);

            publishMarkerGoal();
        });

    auto pl1 = std::placeholders::_1;

    _as = std::make_unique<ActionServer>(_npr,
                                         "plan",
                                         std::bind(&PlannerExecutor::executePlanMotionAction, this, pl1),
                                         false);

    _as->start();

}

bool PlannerExecutor::publishMarkerStart()
{
    bool ret = _planner->checkValid(_q_start);
    _viz_start->publishMarkers(ros::Time::now(), _ps->getCollidingLinks());
    return ret;
}

bool PlannerExecutor::publishMarkerGoal()
{
    bool ret = _planner->checkValid(_q_goal);
    _viz_start->publishMarkers(ros::Time::now(), _ps->getCollidingLinks());
    return ret;
}

void PlannerExecutor::executePlanMotionAction(const cartesio_planning::PlanMotionGoalConstPtr &goal)
{
    std::set<std::string> supported_types = {
      "joint", "pose", "goal_generation", "goal_listener", "goal_state"
    };

    if(supported_types.count(goal->type) == 0)
    {
        cartesio_planning::PlanMotionResult res;
        res.success = false;
        res.message = "invalid goal type +'" + goal->type + "'";
        _as->setAborted(res, res.message);
        return;
    }

    // if we're connected to a robot, the start pose will be the current robot state
    if(_robot)
    {
        // start from current robot pos ref
        if(!goal->start_joint.name.empty())
        {
            cartesio_planning::PlanMotionResult res;
            res.success = false;
            res.message = "cannot specify a start configuration when connected to a robot";
            _as->setAborted(res, res.message);
            return;
        }

        _robot->sense();

        _q_start = _robot->getPositionReferenceFeedback();

    }
    else
    {
        // just use our internal q start (can be set via topic while "jogging")
    }

    // check validity and publish
    if(!publishMarkerStart())
    {
        cartesio_planning::PlanMotionResult res;
        res.success = false;
        res.message = "unable to get initial configuration: initial configuration is invalid";
        _as->setAborted(res, res.message);
        return;
    }

    if(goal->type == "goal_generation")
    {
        // TBD generate q goal
    }
    else if(goal->type == "goal_listener")
    {
        // TBD run goal listener
    }
    else if(goal->type == "goal_state")
    {
        // goal from robot state (srdf)
        _q_goal = _model->getRobotState(goal->goal_robot_state);
    }
    else if(goal->goal_joint.name.empty())
    {
        // use internal goal (can be set via topic while "jogging")
        // nothing to do here
    }
    else
    {
        // use goal from user
        _q_goal = jointStateToQ(goal->goal_joint, _q_start);
    }

    // check validity and publish
    if(!publishMarkerGoal())
    {
        cartesio_planning::PlanMotionResult res;
        res.success = false;
        res.message = "unable to get final configuration: final configuration is invalid";
        _as->setAborted(res, res.message);
        return;
    }

    // now we have start and goal, let's plan
    bool plan_ok = _planner->solve(_q_start,
                                   _q_goal,
                                   goal->planner_timeout,
                                   goal->planner_type.empty() ? "RRTstar" : goal->planner_type);

    if(!plan_ok)
    {
        cartesio_planning::PlanMotionResult res;
        res.success = false;
        res.message = "planner failed";
        _as->setSucceeded(res, res.message);
        return;
    }

    // get path (TODO: interpolation)
    Eigen::MatrixXd path = _planner->getSolutionPath(true);

    cartesio_planning::PlanMotionResult res;
    res.success = true;
    res.message = "planner succeeded";

    // save joint goal
    res.goal_configuration.name = _model->getVNames();
    res.goal_configuration.position.resize(_model->getNv());
    Eigen::VectorXd::Map(res.goal_configuration.position.data(),
                         res.goal_configuration.position.size()) = _model->positionToMinimal(_q_goal);

    // save trajectory
    res.trajectory.joint_names = _model->getVNames();
    res.trajectory.points.resize(path.cols());

    _as->setSucceeded(res, res.message);



}

Eigen::VectorXd PlannerExecutor::jointStateToQ(const sensor_msgs::JointState &js,
                                               const Eigen::VectorXd& q0)
{
    Eigen::VectorXd q = _model->positionToMinimal(q0);

    for(int i = 0; i < js.name.size(); i++)
    {
        int idx = _model->getVIndexFromVName(js.name[i]);

        if(idx < 0)
        {
            throw std::invalid_argument("invalid dof name '" + js.name[i] + "'");
        }

        q[idx] = js.position[i];
    }

    return _model->minimalToPosition(q);
}

