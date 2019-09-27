#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <sensor_msgs/JointState.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "planning_scene_test");
    ros::NodeHandle nh;

    // load robot model from params
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    // create planning scene
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    // planning scene monitor automatically updates planning scene from topics
    planning_scene_monitor::PlanningSceneMonitor monitor(planning_scene, "robot_description");

    // on timer event callback
    auto on_timer_event =
            [&planning_scene,
            &monitor](const ros::TimerEvent& msg)
    {

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        monitor.lockSceneRead();
        planning_scene->checkCollision(collision_request, collision_result);
        monitor.unlockSceneRead();
        ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " collision");
    };

    // every 0.5 sec we check for collisions
    auto timer = nh.createTimer(ros::Duration(0.5), on_timer_event);

    // publish planning scene at 30 Hz (topic is ~/monitored_planning_scene)
    monitor.setPlanningScenePublishingFrequency(30.);

    // this subscribes to /planning_scene
    monitor.startSceneMonitor();

    // this is somehow different from the scene monitor... boh
    monitor.startWorldGeometryMonitor();

    // this subscribes to /joint_states
    monitor.startStateMonitor();

    // this starts monitored planning scene publisher
    monitor.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    ros::spin();
}
