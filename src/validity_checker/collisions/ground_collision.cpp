#include "ground_collision.h"
#include <thread>
#include <chrono>

using namespace XBot::Cartesian::Planning;

GroundCollision::GroundCollision(XBot::ModelInterface::Ptr model):
    _model(model),
    _tol(1e-2),
    _active(false)
{}

void GroundCollision::init()
{
    Eigen::Affine3d T;
    _model->getPose(_link, T);
    _h = (T.translation().transpose() * _axis).value();
}

bool GroundCollision::setAxis(const Eigen::Vector3d axis)
{
    double res = sqrt((axis.norm() - 1) * (axis.norm() - 1));
    if (res > 1e-3)
    {
        std::cout << "[error]: vector given in input is not normalized!" << std::endl;
        return false;
    }

    _axis = axis;
    return true;
}

bool GroundCollision::setLiftedLink(const std::string link)
{
    Eigen::Affine3d T;
    if (!_model->getPose(link, T))
    {
        std::cout << "[error]: link does not exist!" << std::endl;
        return false;
    }

    _link = link;
    return true;
}

void GroundCollision::setActive(bool active)
{
    _active = active;
}

bool GroundCollision::check()
{
    if (!_active)
        return true;
    
    if (!_h)
    {
        std::cout << "GroundCollision not initialized! Set link and/or axis" << std::endl;
        return false;
    }
    
    Eigen::Affine3d T;
    _model->getPose(_link, T);

    double h = (T.translation().transpose() * _axis).value();

    if(_h - h > _tol)
    {
        return false;
    }
    return true;
}

GroundCollisionROS::GroundCollisionROS(GroundCollision::Ptr gc,
                                       XBot::ModelInterface::Ptr model,
                                       ros::NodeHandle& nh):
    _gc(gc),
    _model(model),
    _nh(nh)
{
    _sub = _nh.subscribe("gc", 10, &GroundCollisionROS::setChecker, this);
    _start_model_sub = _nh.subscribe("start/joint_states", 1, &GroundCollisionROS::setStartJointPosition, this);
    _goal_model_sub = _nh.subscribe("goal/joint_states", 1, &GroundCollisionROS::setGoalJointPosition, this);
}

void GroundCollisionROS::setChecker(cartesio_planning::SetGroundCheck::ConstPtr msg)
{
    _gc->setLiftedLink(msg->link);
    _gc->setAxis(Eigen::Vector3d(msg->axis.data()));
    _gc->setActive(msg->active);

    _model->setJointPosition(_q_start);
    _model->update();
    Eigen::Affine3d T;
    _model->getPose(_gc->getLiftedLink(), T);
    double h_start = (T.translation().transpose() * _gc->getAxis()).value();
    _model->setJointPosition(_q_goal);
    _model->update();
    _model->getPose(_gc->getLiftedLink(), T);
    double h_goal = (T.translation().transpose() * _gc->getAxis()).value();

    if (h_start > h_goal)
    {
        _model->setJointPosition(_q_goal);
        _model->update();
    }
    else
    {
        _model->setJointPosition(_q_start);
        _model->update();
    }
    
    if (msg->active)
        _gc->init();
}

void GroundCollisionROS::setStartJointPosition(sensor_msgs::JointState::ConstPtr msg)
{
    auto q = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
    _q_start = q;
}

void GroundCollisionROS::setGoalJointPosition(sensor_msgs::JointState::ConstPtr msg)
{
    auto q = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
    _q_goal = q;
}
