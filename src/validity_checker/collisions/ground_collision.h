#ifndef GROUND_COLLISION_H
#define GROUND_COLLISION_H

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <XBotInterface/ModelInterface.h>
#include <cartesio_planning/SetGroundCheck.h>

#include <ros/ros.h>

namespace XBot { namespace Cartesian { namespace Planning {

class GroundCollision {

public:
    typedef std::shared_ptr<GroundCollision> Ptr;

    GroundCollision(const std::string link,
                    const Eigen::Vector3d axis,
                    XBot::ModelInterface::Ptr model);

    bool setLiftedLink(const std::string link);

    bool setAxis(const Eigen::Vector3d axis);

    std::string getLiftedLink() const {return _link;}

    Eigen::Vector3d getAxis() const {return _axis;}

    bool check();

private:
    void init();

    std::string _link;
    Eigen::Vector3d _axis;
    XBot::ModelInterface::Ptr _model;
    double _tol;
    double _h;
};

class GroundCollisionROS {

public:
    GroundCollisionROS(GroundCollision::Ptr gc,
                       XBot::ModelInterface::Ptr model,
                       ros::NodeHandle& nh);

    void setChecker(cartesio_planning::SetGroundCheck::ConstPtr msg);

private:
    GroundCollision::Ptr _gc;
    XBot::ModelInterface::Ptr _model;
    ros::NodeHandle& _nh;
    ros::Subscriber _sub;
};
} } }

#endif // GROUND_COLLISION_H
