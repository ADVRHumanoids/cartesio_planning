#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>
#include <robot_state_publisher/robot_state_publisher.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

trajectory_msgs::JointTrajectory trj_msg;
int counter;

void jointTrjCallBack(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    counter = 0;
    ROS_INFO("A new trajectory was received!");

    trj_msg = *msg;

    ROS_INFO("#points: %i", trj_msg.points.size());
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "previewer");

    std::string prefix = "planner";

    ros::NodeHandle n(prefix);

    ros::Subscriber joint_trj_sub = n.subscribe("joint_trajectory", 10, jointTrjCallBack);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);


    KDL::Tree kdl_tree;
    kdl_parser::treeFromUrdfModel(model->getUrdf(), kdl_tree);

    std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_state_publisher =
            std::make_shared<robot_state_publisher::RobotStatePublisher>(kdl_tree);

    tf::TransformBroadcaster tf_broadcaster;

    while(ros::ok())
    {

        Eigen::VectorXd q;
        if(!trj_msg.points.empty())
        {
            trajectory_msgs::JointTrajectoryPoint pi = trj_msg.points[counter];
            q = Eigen::Map<Eigen::VectorXd>(pi.positions.data(), pi.positions.size());
        }
        else
            q.setZero(model->getJointNum());

        model->setJointPosition(q);
        model->update();


        XBot::JointNameMap _joint_name_map;
        model->getJointPosition(_joint_name_map);

        std::map<std::string, double> _joint_name_std_map;

        auto predicate = [](const std::pair<std::string, double>& pair)
        {
            return pair.first.find("VIRTUALJOINT") == std::string::npos;
        };

        std::copy_if(_joint_name_map.begin(), _joint_name_map.end(),
                     std::inserter(_joint_name_std_map, _joint_name_std_map.end()),
                     predicate);


        /* Publish world odom */
        Eigen::Affine3d w_T_pelvis;
        w_T_pelvis.setIdentity();
        std::string fb_link = "world";

        if(model->isFloatingBase())
        {
            model->getFloatingBasePose(w_T_pelvis);
            model->getFloatingBaseLink(fb_link);
        }

        tf::Transform transform;
        tf::transformEigenToTF(w_T_pelvis, transform);


        ros::Time t = ros::Time::now();
        robot_state_publisher->publishTransforms(_joint_name_std_map, t, prefix);
        robot_state_publisher->publishFixedTransforms(prefix, true);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform.inverse(),
                                                           t,
                                                           prefix + "/" + fb_link,
                                                           prefix + "/" + "world_odom"));



        ros::spinOnce();

        if(!trj_msg.points.empty())
        {
            (trj_msg.points[counter+1].time_from_start - trj_msg.points[counter].time_from_start).sleep();
            counter++;
            counter = counter % trj_msg.points.size();
        }
        else
            ros::Duration(0.1).sleep();

    }

    return 0;
}
