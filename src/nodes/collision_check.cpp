#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <robot_state_publisher/robot_state_publisher.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include "constraints/self_collision_checker.h"

Eigen::VectorXd q;
XBot::ModelInterface::Ptr model;
XBot::Cartesian::Planning::SelfCollisionChecker::Ptr scc;

void jointCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
    sensor_msgs::JointState jmsg = *msg;
    q = Eigen::Map<Eigen::VectorXd>(jmsg.position.data(), jmsg.position.size());

    model->setJointPosition(q);
    model->update();
    scc->updateCollisionObjects();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_check");

    ros::NodeHandle n("cartesian");

    auto cfg = XBot::ConfigOptionsFromParamServer();
    model = XBot::ModelInterface::getModel(cfg);

    q.setZero(model->getJointNum());

    scc = std::make_shared<XBot::Cartesian::Planning::SelfCollisionChecker>(*model);

    ros::Subscriber joint_trj_sub = n.subscribe("solution", 10, jointCallBack);

    std::string link1 = "LSoftHandLink";
    std::string link2 = "RSoftHandLink";
    std::string link3 = "DWYTorso";

    ros::Rate rate(10);
    while(ros::ok())
    {
        if(scc->inCollision(link1, link2))
            ROS_WARN("%s in collision with %s", link1.c_str(), link2.c_str());
        if(scc->inCollision(link1, link3))
            ROS_WARN("%s in collision with %s", link1.c_str(), link3.c_str());

        ros::spinOnce();
        rate.sleep();
    }
}
