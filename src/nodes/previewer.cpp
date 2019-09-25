#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>
#include <robot_state_publisher/robot_state_publisher.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

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

    tf::TransformListener listener;
    listener.waitForTransform("ci/world", "ci/world_odom", ros::Time(0), ros::Duration(1) );


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


        if(model->isFloatingBase())
        {
            /* Publish world odom */
            Eigen::Affine3d w_T_pelvis;
            w_T_pelvis.setIdentity();
            std::string fb_link = "world";

            model->getFloatingBasePose(w_T_pelvis);
            model->getFloatingBaseLink(fb_link);


            tf::Transform transform;
            tf::transformEigenToTF(w_T_pelvis, transform);



            tf::Transform wp_T_rp;
            Eigen::Affine3d tmp; model->getPose(fb_link, tmp);
            tf::transformEigenToTF(tmp, wp_T_rp);


            tf::StampedTransform wc_T_rc;
            listener.lookupTransform("ci/"+fb_link, "ci/world_odom",
                                     ros::Time(0), wc_T_rc);

            tf::Transform rp_T_rc = wc_T_rc*wp_T_rp;

            ros::Time t = ros::Time::now();

            std::vector<tf::StampedTransform> tfs;
            tfs.push_back(tf::StampedTransform(rp_T_rc, t, "ci/"+fb_link, "planner/world"));
            tfs.push_back(tf::StampedTransform(transform.inverse(),
                                                               t,
                                                               prefix + "/" + fb_link,
                                                               prefix + "/" + "world_odom"));



            robot_state_publisher->publishTransforms(_joint_name_std_map, t, prefix);
            robot_state_publisher->publishFixedTransforms(prefix, true);
            tf_broadcaster.sendTransform(tfs);
        }
        else
        {
            ros::Time t = ros::Time::now();

            std::vector<tf::StampedTransform> tfs;
            tf::Transform I; I.setIdentity();
            tfs.push_back(tf::StampedTransform(I, t, "ci/world", "planner/world"));


            robot_state_publisher->publishTransforms(_joint_name_std_map, t, prefix);
            robot_state_publisher->publishFixedTransforms(prefix, true);
            tf_broadcaster.sendTransform(tfs);
        }






        ros::spinOnce();

        if(!trj_msg.points.empty())
        {
            ros::Duration dt = trj_msg.points[counter+1].time_from_start - trj_msg.points[counter].time_from_start;
            dt.sleep();
            counter++;
            counter = counter % (trj_msg.points.size()-1);
        }
        else
            ros::Duration(0.1).sleep();

    }

    return 0;
}
