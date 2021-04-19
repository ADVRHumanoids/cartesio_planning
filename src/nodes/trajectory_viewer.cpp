#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

int main(int argc, char ** argv)
{
    // initialize ros
    ros::init(argc, argv, "trajectory_viewer");
    ros::NodeHandle nh("planner");

    // retrieve xbot model
    auto opt = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(opt);


    std::shared_ptr<ros::Rate> rate;
    std::vector<Eigen::VectorXd> trj_points;
    int k = 0;
    // on trajectory received callback
    auto on_trj_received = [&trj_points, &k, &rate](const trajectory_msgs::JointTrajectoryConstPtr& msg)
    {
        trj_points.clear();
        for(auto pi : msg->points)
        {
            auto pi_eigen = Eigen::VectorXd::Map(pi.positions.data(),
                                                 pi.positions.size());
            
            trj_points.push_back(pi_eigen);
        }


        rate = std::make_shared<ros::Rate>(1./(msg->points[1].time_from_start.sec + msg->points[1].time_from_start.nsec/1e9)); //We assume constant time along the traj.
        k = 0;
    };

    // joint trajectory subscriber (interpolated trajectory)
    auto sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, on_trj_received);
    
    // raw joint trajectory subscriber
//     auto sub = nh.subscribe<trajectory_msgs::JointTrajectory>("raw_trajectory", 1, on_trj_received);

    // custom robot state publisher
    XBot::Cartesian::Utils::RobotStatePublisher rspub(model);


    ros::Rate fixed_rate(100.);
    while(ros::ok())
    {
        if(rate)
        {
            rate->sleep();

            ros::spinOnce();
            
            // evaluate trajectory at time
            auto qi = trj_points[k];

            // set model accordingly
            model->setJointPosition(qi);
            model->update();

            k += 1;

            // rewind trajectory playback
            if(k >= trj_points.size())
            {
                k = 0;
            }

            // publish model tf
            rspub.publishTransforms(ros::Time::now(), "planner");
        }
        else
        {
            Eigen::VectorXd qhome(model->getJointNum());
            model->getRobotState("home", qhome);
            qhome << 0.0538123, -7.25856e-17,    0.0269701,  1.08748e-16, -7.87239e-07,  -1.2962e-17,  -0.00373849,    -0.393397,   4.8984e-07,     0.711108,    -0.317651,   0.00373866,   0.00373849, -0.393397,  -4.8984e-07,     0.711108,    -0.317651,  -0.00373866,            0, 0,     0.959931,     0.007266,            0,     -1.91986,            0,   -0.523599, 0,     0.959931,   -0.007266,            0,     -1.91986,            0,    -0.523599,            0;

            model->setJointPosition(qhome);
            model->update();
            
            rspub.publishTransforms(ros::Time::now(), "planner");
            fixed_rate.sleep();
            ros::spinOnce();
        }
    }
}
