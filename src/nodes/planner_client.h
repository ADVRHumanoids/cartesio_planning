#ifndef PLANNER_CLIENT_H
#define PLANNER_CLIENT_H

#include <ros/ros.h>
#include <cartesio_planning/CartesioJointStates.h>
#include <cartesio_planning/CartesioPlanner.h>
#include <cartesio_planning/CartesioFrames.h>

#include <XBotInterface/TypedefAndEnums.h>
#include <stdio.h>

class PlannerClient
{
public:
    PlannerClient();

    void setStartState(const XBot::JointNameMap &q);
    void setGoalState(const XBot::JointNameMap &q);
    void callPlanner(const double time, const std::string& planner_type);
    void setContactFrames(std::string action, std::list<std::string> frames_in_contact);

private:
    ros::NodeHandle nh;

    ros::ServiceClient _client_start;
    ros::ServiceClient _client_goal;
    ros::ServiceClient _client_planner;
    ros::ServiceClient _client_frames;
};

#endif // PLANNER_CLIENT_H
