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

    // tf
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("world", "base_link", ros::Time::now(), ros::Duration(3.0));

    // create planning scene
    auto planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

    // planning scene monitor automatically updates planning scene from topics
    planning_scene_monitor::PlanningSceneMonitor monitor(planning_scene, "robot_description");

    // on timer event callback
    auto on_check_collisions_event =
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

    // on timer event callback
    auto on_joint_state_received =
            [&monitor,
            &tf_listener](const sensor_msgs::JointStateConstPtr& msg)
    {
        monitor.lockSceneWrite();

        for(int i = 6; i < msg->name.size(); i++)
        {
            monitor.getPlanningScene()->getCurrentStateNonConst().setJointPositions(msg->name[i], {msg->position[i]});
        }

        tf::StampedTransform tf;
        tf_listener.lookupTransform("world", "base_link", ros::Time(0), tf);

        std::vector<double> qv = {
            tf.getOrigin().x(),
            tf.getOrigin().y(),
            tf.getOrigin().z(),
            tf.getRotation().x(),
            tf.getRotation().y(),
            tf.getRotation().z(),
            tf.getRotation().w()
        };

        monitor.getPlanningScene()->getCurrentStateNonConst().setJointPositions("reference", qv);
        monitor.unlockSceneWrite();

        monitor.triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE);
    };

    // every 0.5 sec we check for collisions
    auto collisions_timer = nh.createTimer(ros::Duration(0.5), on_check_collisions_event);

    // every 0.01 sec we update robot state
    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, on_joint_state_received);

    // publish planning scene at 30 Hz (topic is ~/monitored_planning_scene)
    monitor.setPlanningScenePublishingFrequency(30.);

    // this subscribes to /planning_scene
    monitor.startSceneMonitor();

    // this is somehow different from the scene monitor... boh
    monitor.startWorldGeometryMonitor();

    // this subscribes to /joint_states
//    monitor.startStateMonitor(); // we do this manually in the above callback "on_joint_state_received"




    // this starts monitored planning scene publisher
    monitor.startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    ros::spin();
}
