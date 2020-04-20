#include <ros/ros.h>
#include <tf/tf.h>
#include <interactive_markers/interactive_marker_server.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

octomap_msgs::Octomap map;


void callback ( const octomap_msgs::OctomapPtr& msg ) {
    map.header = msg->header;
    map.binary = msg->binary;
    map.resolution = msg->resolution;
    map.id = msg->id;
    map.data = msg->data;
}

octomap_msgs::OctomapWithPose generateMessage ( octomap_msgs::Octomap inputMap ) {
    octomap_msgs::OctomapWithPose msg2;

    // Assign the same header
    msg2.header = inputMap.header;

    // Octree  frame coincident with the header frame
    msg2.origin.position.x = 0;
    msg2.origin.position.y = 0;
    msg2.origin.position.z = 0;
    msg2.origin.orientation.x = 0;
    msg2.origin.orientation.y = 0;
    msg2.origin.orientation.z = 0;
    msg2.origin.orientation.w = 1;

    // Assign the map
    msg2.octomap.header = inputMap.header;
    msg2.octomap.binary = inputMap.binary;
    msg2.octomap.id = inputMap.id;
    msg2.octomap.resolution = inputMap.resolution;
    msg2.octomap.data = inputMap.data;

    return msg2;
}

int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "octomap_collision_node" );
    ros::NodeHandle nh;
    ros::Publisher octomap_collision_publisher = nh.advertise<octomap_msgs::OctomapWithPose> ( "planner/octomap_collision_objects",1000 );

    // Subsribe the octomap topic
    ros::Subscriber octomap_collision_subscriber = nh.subscribe ( "/octomap_binary", 1000, callback );

    ros::Rate rate ( 10 );
    while ( ros::ok() ) {
        // Create the msg
        octomap_msgs::OctomapWithPose map_to_publish;
        map_to_publish = generateMessage ( map );

        // Publish the service
        octomap_collision_publisher.publish ( map_to_publish );
        ros::spinOnce();
        rate.sleep();
    }

}
