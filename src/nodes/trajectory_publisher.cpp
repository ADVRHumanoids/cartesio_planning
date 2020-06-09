#include <ros/ros.h>
#include <xbot_msgs/JointCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;
    std::vector<Eigen::VectorXd> traj;
    std::shared_ptr<ros::Rate> rate;
    
    ros::Publisher pub = nh.advertise<xbot_msgs::JointCommand>("xbotcore/command", 1000);
      
    auto callback = [&traj, &rate](const trajectory_msgs::JointTrajectoryConstPtr& msg)
    {   
        traj.clear();
        for (auto x : msg->points)
        {
            auto x_eigen = Eigen::VectorXd::Map(x.positions.data(), x.positions.size());
            traj.push_back(x_eigen);
        }
        
        rate = std::make_shared<ros::Rate>(1./(msg->points[1].time_from_start.nsec/1e9));
    };
    
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("planner/joint_trajectory", 1000, callback);
    
    ros::Rate fixed_rate(100.);
    while (ros::ok())
    {
        
    }
}