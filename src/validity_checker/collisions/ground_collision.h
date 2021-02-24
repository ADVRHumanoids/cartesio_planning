#ifndef GROUND_COLLISION_H
#define GROUND_COLLISION_H

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <XBotInterface/ModelInterface.h>
#include <cartesio_planning/SetGroundCheck.h>
#include <sensor_msgs/JointState.h>

#include <ros/ros.h>

namespace XBot { namespace Cartesian { namespace Planning {

class GroundCollision {

public:
    typedef std::shared_ptr<GroundCollision> Ptr;

    GroundCollision(XBot::ModelInterface::Ptr model);

    bool setLiftedLink(const std::string link);

    bool setAxis(const Eigen::Vector3d axis);
    
    void setActive(bool active);

    std::string getLiftedLink() const {return _link;}

    Eigen::Vector3d getAxis() const {return _axis;}
    
    bool isActive() const {return _active;}

    bool check();
    
    void init();

private:
    std::string _link;
    Eigen::Vector3d _axis;
    XBot::ModelInterface::Ptr _model;
    double _tol;
    double _h;
    bool _active;
};

class GroundCollisionROS {

public:
    GroundCollisionROS(GroundCollision::Ptr gc,
                       XBot::ModelInterface::Ptr model,
                       ros::NodeHandle& nh);

    void setChecker(cartesio_planning::SetGroundCheck::ConstPtr msg);
    
    void setStartJointPosition(sensor_msgs::JointState::ConstPtr msg);
    void setGoalJointPosition(sensor_msgs::JointState::ConstPtr msg);

private:
    GroundCollision::Ptr _gc;
    XBot::ModelInterface::Ptr _model;
    ros::NodeHandle& _nh;
    ros::Subscriber _sub;
    ros::Subscriber _start_model_sub, _goal_model_sub;
    Eigen::VectorXd _q_start, _q_goal;
};
} } }

#endif // GROUND_COLLISION_H
