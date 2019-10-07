#include "validity_checker/collisions/planning_scene_wrapper.h"

#include <sensor_msgs/JointState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <RobotInterfaceROS/ConfigFromParam.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "planning_scene_test");
    ros::NodeHandle nh;

    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(cfg);

    std::string floating_base_name = "";
    model->getFloatingBaseLink(floating_base_name);

    std::cout << "floating_base_name is " << floating_base_name << std::endl;

    // tf
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("world",
                                 floating_base_name,
                                 ros::Time::now(),
                                 ros::Duration(3.0));

    XBot::Cartesian::Planning::PlanningSceneWrapper collision_detection(model);
    collision_detection.startMonitor();

    // on timer event callback
    auto on_check_collisions_event =
            [&collision_detection](const ros::TimerEvent& msg)
    {
        collision_detection.update();
        bool is_colliding = collision_detection.checkCollisions();
        ROS_INFO_STREAM("Test 1: Current state is " << (is_colliding ? "in" : "not in") << " collision");
    };

    // on timer event callback
    auto on_joint_state_received =
            [model, floating_base_name,
            &tf_listener](const sensor_msgs::JointStateConstPtr& msg)
    {

        tf::StampedTransform tf;
        tf_listener.lookupTransform("world", floating_base_name, ros::Time(0), tf);

        Eigen::Affine3d w_T_fb;
        tf::transformTFToEigen(tf, w_T_fb);

        model->setFloatingBasePose(w_T_fb.inverse());

        XBot::JointNameMap jname_map;

        for(int i = 0; i < msg->name.size(); i++)
        {
            jname_map[msg->name[i]] = msg->position[i];
        }

        model->setJointPosition(jname_map);
        model->update();

    };


    // every 0.5 sec we check for collisions
    auto collisions_timer = nh.createTimer(ros::Duration(0.5), on_check_collisions_event);

    // every 0.01 sec we update robot state
    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 10, on_joint_state_received);

    ros::spin();
}
