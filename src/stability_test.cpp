#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include "validity_checker/stability/stability_detection.h"
#include <visualization_msgs/Marker.h>

using namespace XBot::Cartesian::Planning;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "stability_test");
    ros::NodeHandle nh;



    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

    ConvexHullStability ch(model);
    ConvexHullStability::PolygonFrames polyframes;
    polyframes.push_back("l_foot_lower_left_link");
    polyframes.push_back("l_foot_lower_right_link");
    polyframes.push_back("l_foot_upper_left_link");
    polyframes.push_back("l_foot_upper_right_link");
    polyframes.push_back("r_foot_lower_left_link");
    polyframes.push_back("r_foot_lower_right_link");
    polyframes.push_back("r_foot_upper_left_link");
    polyframes.push_back("r_foot_upper_right_link");
    ch.setPolygonFrames(polyframes);

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "convex_hull", 0 );


    auto on_js_received = [&ch, model, vis_pub](const sensor_msgs::JointStateConstPtr& msg)
    {
        Eigen::VectorXd q(model->getJointNum()); q.setZero();
        for(int i = 0; i < msg->name.size(); i++)
            q[i] = msg->position[i];


        model->setJointPosition(q);
        model->update();


        PlanarInclusionDetectionBase::Polygon poly;
        if(ch.getConvexHull(poly))
        {
            bool stable = ch.checkStability();

            visualization_msgs::Marker ch_marker;

            ch_marker.header.frame_id = "ci/world_odom";
            ch_marker.header.stamp = ros::Time::now();
            ch_marker.ns = "convex_hull";
            ch_marker.id = 0;
            ch_marker.type = visualization_msgs::Marker::LINE_STRIP;
            ch_marker.action = visualization_msgs::Marker::ADD;

            Eigen::Vector3d com;
            model->getCOM(com);

            geometry_msgs::Point p;
            ch_marker.points.clear();
            for(auto pp : poly){
                p.x = pp[0] + com[0];
                p.y = pp[1] + com[1];

                ch_marker.points.push_back(p);
            }

            p.x = poly.at(0)[0] + com[0];
            p.y = poly.at(0)[1] + com[1];
            ch_marker.points.push_back(p);

            if(stable)
            {
                ch_marker.color.a = 1.0;
                ch_marker.color.r = 0.0;
                ch_marker.color.g = 1.0;
                ch_marker.color.b = 0.0;
            }
            else
            {
                ch_marker.color.a = 1.0;
                ch_marker.color.r = 1.0;
                ch_marker.color.g = 0.0;
                ch_marker.color.b = 0.0;
            }

            ch_marker.scale.x = 0.01;

            vis_pub.publish(ch_marker);
        }





    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, on_js_received);

    ros::spin();

    return 0;
}
