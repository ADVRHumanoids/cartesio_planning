#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>

trajectory_msgs::JointTrajectory trj_msg;
int counter = 0;

void jointTrjCallBack(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    ROS_INFO("A new trajectory was received!");

    trj_msg = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "previewer");

    ros::NodeHandle n("planner");

    ros::Subscriber joint_trj_sub = n.subscribe("joint_trajectory", 10, jointTrjCallBack);

    if(!trj_msg.points.empty())
    {
        while(ros::ok())
        {
            Eigen::Map<Eigen::VectorXd> q(trj_msg.points[counter].positions.data(),
                                          trj_msg.points[counter].positions.size());
            std::cout<<"q @"<<counter<<": "<<q.transpose()<<std::endl;

            ros::Duration(trj_msg.points[counter].time_from_start).sleep();
            counter = counter++;
            counter = counter % trj_msg.points.size();

            ros::spinOnce();
        }
    }

    ros::spin();

    return 0;
}
