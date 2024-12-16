#ifndef CP_ROBOT_VIZ_H
#define CP_ROBOT_VIZ_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <xbot2_interface/xbotinterface2.h>

#include "../common/types.h"


namespace XBot::Cartesian::Planning {

/**
 * @brief The RobotViz class is used to publish a marker array with all the meshes (collision & visual) of the robot
 * in a uniform color.
 */
class RobotViz
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(RobotViz);

    typedef Eigen::Vector4d color; // rgba

    /**
     * @brief RobotViz
     * @param model model of the robot to publish as marker
     * @param topic_name topic of the published marker
     * @param nh to retrieve the namespace of the topic
     * @param rgba color of the robot published
     */
    RobotViz(ModelInterface::ConstPtr model,
             std::string topic_name,
             ros::NodeHandle nh = ros::NodeHandle("~"),
             std::optional<color> rgba = std::nullopt);

    RobotViz(ModelInterface::ConstPtr model,
             std::string topic_name,
             std::optional<color> rgba = std::nullopt);

    /**
     * @brief setPrefix
     * @param prefix added to header frame id of the marker (eg "planner/")
     */
    void setPrefix(const std::string& prefix);

    /**
     * @brief getPrefix
     * @return actual prefix
     */
    std::string getPrefix();

    /**
     * @brief setRGBA
     * @param rgba [Red, Green, Blue, Alpha]
     */
    void setRGBA(const color& rgba);

    /**
     * @brief publishMarkers of the robot with a time
     * @param time
     * @param red_links these links will be publisged with the _reserved_color
     */
    void publishMarkers(const ros::Time& time, const std::vector<std::string>& red_links);

private:

    /**
     * @brief _reserved_color for the self collision
     */
    const color _reserved_color;

    XBot::ModelInterface::ConstPtr _model;
    ros::NodeHandle _nh;
    ros::Publisher collision_robot_pub;
    std::string _prefix;
    color _rgba;

    static Eigen::Affine3d toAffine3d(const urdf::Pose& p);

};

}


#endif // ROBOT_VIZ_H
