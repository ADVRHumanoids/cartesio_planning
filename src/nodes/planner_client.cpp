#include "planner_client.h"

PlannerClient::PlannerClient() :
    nh("planner")
{
    _client_start = nh.serviceClient<cartesio_planning::CartesioJointStates>("start_pose/joint_states");
    _client_goal = nh.serviceClient<cartesio_planning::CartesioJointStates>("goal_pose/joint_states");
    _client_planner = nh.serviceClient<cartesio_planning::CartesioPlanner>("compute_plan");
    _client_frames = nh.serviceClient<cartesio_planning::CartesioFrames>("contact_frames");
}

void PlannerClient::setStartState(const XBot::JointNameMap &q)
{
    cartesio_planning::CartesioJointStates srv;
    for(auto elem : q) {
        srv.request.joint_states.name.push_back(elem.first);
        srv.request.joint_states.position.push_back(elem.second);
    }

    if (_client_start.call(srv))
    {
        ROS_INFO("Sent start pose.");
    }
    else
    {
        ROS_ERROR("Failed to send start pose.");
    }
}

void PlannerClient::setGoalState(const XBot::JointNameMap &q)
{
    cartesio_planning::CartesioJointStates srv;

    for(auto elem : q) {
        srv.request.joint_states.name.push_back(elem.first);
        srv.request.joint_states.position.push_back(elem.second);
    }

    if (_client_goal.call(srv))
    {
        ROS_INFO("Sent goal pose.");
    }
    else
    {
        ROS_ERROR("Failed to send goal pose.");
    }
}

void PlannerClient::callPlanner(const double time, const std::string& planner_type)
{
    cartesio_planning::CartesioPlanner srv;
    srv.request.time = time;
    srv.request.planner_type = planner_type;

    if (_client_planner.call(srv))
    {
        ROS_INFO("Sent planning request.");
    }
    else
    {
        ROS_ERROR("Failed to send planning request.");
    }
}

void PlannerClient::setContactFrames(std::string action, std::list<std::string> frames_in_contact)
{
    cartesio_planning::CartesioFrames srv;
    srv.request.action = action;

    for(auto elem : frames_in_contact)
    {
        srv.request.frames_in_contact.push_back(elem);
    }

    if (_client_frames.call(srv))
    {
        ROS_INFO("Sent contact frames.");
    }
    else
    {
        ROS_ERROR("Failed to send contact frames.");
    }
}
