#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>


Eigen::Affine3d toAffine3d(const urdf::Pose& p)
{
    Eigen::Affine3d T;

    T.translation()[0] = p.position.x;
    T.translation()[1] = p.position.y;
    T.translation()[2] = p.position.z;

    T.linear() = Eigen::Matrix3d(Eigen::Quaterniond(p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z));
    return T;
}

visualization_msgs::MarkerArray createRobotMarkerArray(const XBot::ModelInterface& model, const std::vector<std::string> contact_links)
{
    visualization_msgs::MarkerArray markers;

    std::string bl; model.getFloatingBaseLink(bl);

    ros::Time t = ros::Time::now();

    std::vector<urdf::LinkSharedPtr> links;
    model.getUrdf().getLinks(links);

    int id = 0;
    for(auto link : links)
    {
        if(link->collision)
        {
            if(link->collision->geometry->type == urdf::Geometry::MESH)
            {

                visualization_msgs::Marker marker;

                marker.header.frame_id = "ci/"+bl;
                marker.header.stamp = t;
                marker.ns = "collision_robot";
                marker.id = id;

                marker.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker.action = visualization_msgs::Marker::ADD;

                Eigen::Affine3d pose; model.getPose(link->name, bl, pose);

                pose = pose*toAffine3d(link->collision->origin);

                marker.pose.position.x = pose.translation()[0];
                marker.pose.position.y = pose.translation()[1];
                marker.pose.position.z = pose.translation()[2];
                Eigen::Quaterniond q(pose.linear());
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();

                boost::shared_ptr<urdf::Mesh> mesh =
                        boost::static_pointer_cast<urdf::Mesh>(link->collision->geometry);

                marker.mesh_resource = mesh->filename;
                marker.scale.x = mesh->scale.x;
                marker.scale.y = mesh->scale.y;
                marker.scale.z = mesh->scale.z;



                if(std::find(contact_links.begin(), contact_links.end(), link->name) != contact_links.end())
                {
                    marker.color.a = 1.0;
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }
                else
                {
                    marker.color.a = 1.0;
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }


                markers.markers.push_back(marker);

                id++;
            }
        }
    }
    return markers;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "planning_scene_test");
    ros::NodeHandle nh;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);


    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    ros::Publisher collision_robot_pub = nh.advertise<visualization_msgs::MarkerArray>( "collision_robot", 0 );

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
    auto on_js_received = [&collision_robot_pub, &model, &current_state, &planning_scene, &has_joint_name](const sensor_msgs::JointStateConstPtr& msg)
    {
        for(int i = 0; i < msg->name.size(); i++)
        {
            if(!has_joint_name(msg->name[i]))
            {
                continue;
            }

            current_state.setJointPositions(msg->name[i], {msg->position[i]});
        }

        model->setJointPosition(Eigen::VectorXd::Map(msg->position.data(), msg->position.size()));
        model->update();

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        planning_scene.checkSelfCollision(collision_request, collision_result);

        std::vector<std::string> contact_links;
        if(collision_result.collision)
            planning_scene.getCollidingLinks(contact_links);


        collision_robot_pub.publish(createRobotMarkerArray(*model, contact_links));


    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, on_js_received);

    ros::spin();
}
