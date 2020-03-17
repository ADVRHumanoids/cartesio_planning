#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <XBotInterface/ConfigOptions.h>
#include <urdf_parser/urdf_parser.h>
#include <RobotInterfaceROS/ConfigFromParam.h>


int main ( int argc, char** argv ) {
    ros::init ( argc, argv, "state_publisher" );
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState> ( "joint_states", 1 );
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate ( 30 );

    const double degree = M_PI/180;

    // robot state
    double y = 0, x=0, rot=0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "base_link";

    while ( ros::ok() ) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize ( 3 );
        joint_state.position.resize ( 3 );
        joint_state.name[0] ="j_x";
        joint_state.position[0] = x;
        joint_state.name[1] ="j_y";
        joint_state.position[1] = y;
        joint_state.name[2] ="j_rot";
        joint_state.position[2] = rot;


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.1;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw ( rot );

        //send the joint state and transform
        joint_pub.publish ( joint_state );
        broadcaster.sendTransform ( odom_trans );

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
