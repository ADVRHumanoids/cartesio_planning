#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>

#include "planner/trajectory_interpolation.h"

int main(int argc, char ** argv)
{
    // initialize ros
    ros::init(argc, argv, "trajectory_viewer");
    ros::NodeHandle nh("planner");

    // retrieve xbot model
    auto opt = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(opt);

    // construct trajectory interpolator
    TrajectoryInterpolation trj(model->getJointNum());

    double time = 0; // playback trajectory time

    // on trajectory received callback
    auto on_trj_received = [&trj, &time](const trajectory_msgs::JointTrajectoryConstPtr& msg)
    {
        std::vector<Eigen::VectorXd> trj_points;
        for(auto pi : msg->points)
        {
            auto pi_eigen = Eigen::VectorXd::Map(pi.positions.data(),
                                                 pi.positions.size());

            trj_points.push_back(pi_eigen);
        }

        trj.compute(trj_points); // this optimizes the trajectory
        time = 0; // initialize the playback time
    };

    // joint trajectory subscriber
    auto sub = nh.subscribe<trajectory_msgs::JointTrajectory>("joint_trajectory", 1, on_trj_received);

    // custom robot state publisher
    XBot::Cartesian::Utils::RobotStatePublisher rspub(model);

    ros::Rate rate(100.);


    while(ros::ok())
    {
        rate.sleep();

        ros::spinOnce();

        // we haven't received anything yet
        if(!trj.isValid())
        {
            continue;
        }

        // evaluate trajectory at time
        auto qi = trj.evaluate(time);

        // set model accordingly
        model->setJointPosition(qi);
        model->update();

        // increment time
        time += rate.expectedCycleTime().toSec();

        // rewind trajectory playback
        if(time > trj.getTrajectoryEndTime())
        {
            time = 0.0;
        }

        // publish model tf
        rspub.publishTransforms(ros::Time::now(), "planner");

    }
}
