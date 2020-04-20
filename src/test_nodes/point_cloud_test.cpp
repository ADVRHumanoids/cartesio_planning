#include <ros/ros.h>
#include <ros/node_handle.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>


double x = 0.5;


int main ( int argc, char ** argv ) {
    ros::init ( argc, argv, "point_cloud_generator" );
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ( "/velodyne_points",1000 );

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud ( new pcl::PointCloud<pcl::PointXYZ> );

    ros::Rate rate ( 10 );
    while ( ros::ok() ) {
        double res = 0.05;
        double length = 0.3;
        std::vector<double> y {-0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15};
        std::vector<double> z {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3};

        point_cloud->header.frame_id = "planner/world";
        point_cloud->header.stamp = pcl_conversions::toPCL ( ros::Time::now() );

        point_cloud->height = 1;
        point_cloud->width = 7 * z.size();
        point_cloud->points.resize ( point_cloud->height * point_cloud->width );

        int j = 0;

        for ( int i = 0; i < z.size(); i++ ) {
            for ( int k = 0; k < y.size(); k++ ) {
                point_cloud->points[j].x = x;
                point_cloud->points[j].y = y[k];
                point_cloud->points[j].z = z[i];
                j++;
            }
        }

        pub.publish ( point_cloud );

        ros::spinOnce();

        rate.sleep();

    }


}
