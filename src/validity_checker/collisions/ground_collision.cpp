#include "ground_collision.h"

using namespace XBot::Cartesian::Planning;

GroundCollision::GroundCollision(const std::string link,
                                 const Eigen::Vector3d axis,
                                 XBot::ModelInterface::Ptr model):
    _link(link),
    _axis(axis),
    _model(model),
    _tol(1e-2)
{
    if(!_link.empty())
    {
        init();
    }
}

void GroundCollision::init()
{
    Eigen::Affine3d T;
    if (!_model->getPose(_link, T))
    {
        std::cout << "[error]: link does not exist! use setLiftedLink(std::string) to change it..." << std::endl;
    }
    double res = sqrt((_axis.norm() - 1) * (_axis.norm() - 1));
    if (res > 1e-3)
    {
        std::cout << "[error]: vector given in input is not normalized! use setAxis(Eigen::Vector3d) to change it..." << std::endl;
    }

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
    init();
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
    init();
    return true;
}

bool GroundCollision::check()
{
    if (!_h)
    {
        std::cout << "GroundCollision not initialized! Set link and/or axis" << std::endl;
        return false;
    }
    Eigen::Affine3d T;
    _model->getPose(_link, T);

    double h = (T.translation().transpose() * _axis).value();

    if(h - _h > _tol)
        return false;

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
}

void GroundCollisionROS::setChecker(cartesio_planning::SetGroundCheck::ConstPtr msg)
{
    _gc->setLiftedLink(msg->link);
    _gc->setAxis(Eigen::Vector3d(msg->axis.data()));
}



