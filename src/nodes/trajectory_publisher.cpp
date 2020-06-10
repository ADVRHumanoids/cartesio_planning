#include <ros/ros.h>
#include <xbot_msgs/JointCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Dense>



int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle nh;
    std::vector<Eigen::VectorXd> traj;
    std::vector<std::string> jname;
    std::shared_ptr<ros::Rate> rate;
    xbot_msgs::JointCommand jcom;
    double dt;
    
    ros::Publisher pub = nh.advertise<xbot_msgs::JointCommand>("xbotcore/command", 1000);
      
    auto callback = [&traj, &rate, &jname, &dt](const trajectory_msgs::JointTrajectoryConstPtr& msg)
    {   
        traj.clear();
        jname.clear();
        for (auto x : msg->points)
        {
            auto x_eigen = Eigen::VectorXd::Map(x.positions.data(), x.positions.size());
            traj.push_back(x_eigen);
        }
        
        for (auto x : msg->joint_names)
            jname.push_back(x);
        
        rate = std::make_shared<ros::Rate>(1./(msg->points[1].time_from_start.nsec/1e9));
        dt = msg->points[1].time_from_start.nsec/1e9;
    };
    
    ros::Subscriber sub = nh.subscribe<trajectory_msgs::JointTrajectory>("planner/joint_trajectory", 1000, callback);
    
    ros::Rate fixed_rate(100.);
    while (ros::ok())
    {
        if (rate)
        {
            for (int j = 0; j < traj.size() - 1; j++)
            {
                std::vector<double> vect;
                for (int k = 0; k < traj[j].size(); k++)
                    vect.push_back(traj[j](k));
                jcom.name.assign(jname.begin() + 6, jname.end());
                jcom.position.assign(vect.begin() + 6, vect.end());
                jcom.velocity.assign(traj[j].size()-6, 0);
                vect.clear();
                jcom.ctrl_mode.assign(traj[j].size()-6, 1);
                jcom.header.stamp = ros::Time::now();
                
//                 std::vector<std::string>::iterator it = std::find(jname.begin(), jname.end(), "j_wheel_1");
//                 unsigned int index = it - jname.begin();
//                 jcom.velocity[index] = -(traj[j+1](index) - traj[j][index])/(dt);
//                 jcom.ctrl_mode[index] = 2;
//                 
//                 it = std::find(jname.begin(), jname.end(), "j_wheel_2");
//                 index = it - jname.begin();
//                 jcom.velocity[index] = -(traj[j+1](index) - traj[j][index])/(dt);
//                 jcom.ctrl_mode[index] = 2;
//                 
//                 it = std::find(jname.begin(), jname.end(), "j_wheel_3");
//                 index = it - jname.begin();
//                 jcom.velocity[index] = -(traj[j+1](index) - traj[j][index])/(dt);
//                 jcom.ctrl_mode[index] = 2;
//                 
//                 it = std::find(jname.begin(), jname.end(), "j_wheel_4");
//                 index = it - jname.begin();
//                 jcom.velocity[index] = -(traj[j+1](index) - traj[j][index])/(dt);
//                 jcom.ctrl_mode[index] = 2;
                
                pub.publish(jcom);
                
                rate->sleep();
                ros::spinOnce();
            }
        }
        else
        {
            fixed_rate.sleep();
            ros::spinOnce();
        }
    }
}