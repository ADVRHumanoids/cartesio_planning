#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <utils/robot_viz.h>

int main ( int argc, char ** argv ) {
    ros::init ( argc, argv, "self_collision_robot" );
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader robot_model_loader ( "robot_description" );

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene ( kinematic_model );

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel ( cfg );


    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    XBot::Cartesian::Planning::RobotViz robot_viz ( model,"collision_robot", nh, Eigen::Vector4d ( 0.0,0.0,1.0,0.5 ) );
    robot_viz.setPrefix ( "ci/" );

    // function to check if a joint name is defined inside the current state
    // (useful to skip virtual joints, moveit uses quaterion)
    auto has_joint_name = [&current_state] ( std::string jname ) {
        auto it = std::find ( current_state.getVariableNames().begin(),
                              current_state.getVariableNames().end(),
                              jname );

        return it != current_state.getVariableNames().end();
    };

    // on joint state received callback
    auto on_js_received = [&robot_viz, &model, &current_state, &planning_scene, &has_joint_name] ( const sensor_msgs::JointStateConstPtr& msg ) {
        for ( int i = 0; i < msg->name.size(); i++ ) {
            if ( !has_joint_name ( msg->name[i] ) ) {
                continue;
            }

            current_state.setJointPositions ( msg->name[i], {msg->position[i]} );
        }

        model->setJointPosition ( Eigen::VectorXd::Map ( msg->position.data(), msg->position.size() ) );
        model->update();

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene.checkSelfCollision ( collision_request, collision_result );

        std::vector<std::string> contact_links;
        if ( collision_result.collision )
            planning_scene.getCollidingLinks ( contact_links );


        robot_viz.publishMarkers ( ros::Time::now(), contact_links );


    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState> ( "joint_states", 1, on_js_received );

    ros::spin();
}
