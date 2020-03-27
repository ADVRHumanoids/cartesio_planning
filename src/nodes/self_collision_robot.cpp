#include "validity_checker/collisions/planning_scene_wrapper.h"
#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <utils/robot_viz.h>

using namespace XBot::Cartesian;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "self_collision_robot");
    ros::NodeHandle nh;

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

    Planning::PlanningSceneWrapper ps(model);
    Planning::RobotViz rviz(model, "self_collision_robot", nh);

    auto on_js_received = [model, &ps, &rviz](sensor_msgs::JointStateConstPtr msg)
    {
          XBot::JointNameMap jmap;
          for(int i = 0; i < msg->name.size(); i++)
          {
              jmap[msg->name[i]] = msg->position.at(i);
          }
          model->setJointPosition(jmap);
          model->update();
          ps.update();

          rviz.publishMarkers(msg->header.stamp, ps.getCollidingLinks());

    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, on_js_received);

    ros::spin();
}
