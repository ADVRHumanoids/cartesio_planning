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

void PlannerClient::callPlanner(const double time,
                                std::string& planner_type,
                                const double interpolation_time,
                                const std::string trajectory_space,
                                const std::vector<std::string> distal_links,
                                const std::vector<std::string> base_links)
{
    cartesio_planning::CartesioPlanner srv;
    srv.request.time = time;
    srv.request.planner_type = planner_type;
    srv.request.interpolation_time = interpolation_time;
    srv.request.trajectory_space = trajectory_space;

    for (auto elem : base_links)
    {
        srv.request.base_links.push_back(elem);
    }

    for (auto elem : distal_links)
    {
        srv.request.distal_links.push_back(elem);
    }

    if (_client_planner.call(srv))
    {
        ROS_INFO("Sent planning request.");
        if (!srv.response.joint_trajectory.points.empty())
        {
            int j = 0;
            for (auto point_elem : srv.response.joint_trajectory.points)
            {
                for (auto joint_elem : srv.response.joint_trajectory.joint_names)
                {
                    _joint_trajectory[joint_elem].resize(srv.response.joint_trajectory.points.size());
                }
                for (int i = 0; i < srv.response.joint_trajectory.joint_names.size(); i++)
                {
                    _joint_trajectory[srv.response.joint_trajectory.joint_names.at(i)](j) = point_elem.positions.at(i);
                }
                j++;
            }
        }

        if (!srv.response.cartesian_trajectory.empty())
        {
            for (int i = 0; i < srv.response.cartesian_trajectory.size(); i++)
            {
                std::string base_elem = srv.response.cartesian_trajectory[i].base_link;
                std::string distal_elem = srv.response.cartesian_trajectory[i].distal_link;

                for (auto elem : srv.response.cartesian_trajectory[i].frames)
                {
                    Eigen::Affine3d temp_pose;
                    tf::poseMsgToEigen(elem, temp_pose);
                    _cartesian_trajectory[std::make_pair(base_elem, distal_elem)].push_back(temp_pose);
                }
            }
        }
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

std::map<std::string, Eigen::VectorXd> PlannerClient::getJointTrajectory()
{
    if (!_joint_trajectory.empty())
        return _joint_trajectory;
}

std::map<std::pair<std::string, std::string>, std::vector<Eigen::Affine3d>> PlannerClient::getCartesianTrajectory()
{
    if (!_cartesian_trajectory.empty())
        return _cartesian_trajectory;
    else
    {
        ROS_ERROR("There is no Cartesian trajectory, you asked for a Joint trajectory.");
    }
}
