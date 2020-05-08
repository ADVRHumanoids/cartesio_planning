#include "foot_step_planner.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "cartesio_planning_node");
    ros::NodeHandle nhpr("~");

    XBot::Cartesian::FootStepPlanner exec;

    ros::Rate rate(nhpr.param("rate", 30.));

    while(ros::ok())
    {
        exec.run();
        rate.sleep();
    }
}