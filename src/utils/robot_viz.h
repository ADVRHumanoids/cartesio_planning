#ifndef _ROBOT_VIZ_H_
#define _ROBOT_VIZ_H_

#include <XBotInterface/ModelInterface.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>


namespace XBot { namespace Cartesian { namespace Planning {

class RobotViz
{

public:

    typedef Eigen::Vector4d color; //rgba

    typedef std::shared_ptr<RobotViz> Ptr;

    const color _reserved_color = (color() << 1.0, 0.0, 0.0, 1.0).finished();

    RobotViz(const XBot::ModelInterface::ConstPtr model,
             const std::string& topic_name,
             ros::NodeHandle& nh,
             const color& rgba = (color() << 0.0, 1.0, 0.0, 0.5).finished());

    void setPrefix(const std::string& prefix);

    std::string getPrefix();

    void setRGBA(double R, double G, double B, double A);

    void publishMarkers(const ros::Time& time, const std::vector<std::string>& red_links);

private:

    XBot::ModelInterface::ConstPtr _model;
    ros::NodeHandle _nh;
    ros::Publisher collision_robot_pub;
    std::string _prefix;
    color _rgba;

    static Eigen::Affine3d toAffine3d(const urdf::Pose& p);

};

}
}
}

#endif
