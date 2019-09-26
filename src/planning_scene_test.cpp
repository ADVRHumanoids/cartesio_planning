#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "planning_scene_test");
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();

    // function to check if a joint name is defined inside the current state
    // (useful to skip virtual joints, moveit uses quaterion)
    auto has_joint_name = [&current_state](std::string jname)
    {
        auto it = std::find(current_state.getVariableNames().begin(),
                            current_state.getVariableNames().end(),
                            jname);

        return it != current_state.getVariableNames().end();
    };

    // on joint state received callback
    auto on_js_received = [&current_state, &planning_scene, &has_joint_name](const sensor_msgs::JointStateConstPtr& msg)
    {
        for(int i = 0; i < msg->name.size(); i++)
        {
            if(!has_joint_name(msg->name[i]))
            {
                continue;
            }

            current_state.setJointPositions(msg->name[i], {msg->position[i]});
        }

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene.checkSelfCollision(collision_request, collision_result);
        ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, on_js_received);

    ros::spin();
}
