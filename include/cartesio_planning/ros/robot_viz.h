#ifndef CP_ROBOT_VIZ_H
#define CP_ROBOT_VIZ_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
             rclcpp::Node::SharedPtr node = nullptr,
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
    void publishMarkers(const rclcpp::Time& time, const std::vector<std::string>& red_links);

    /**
     * @brief getNode
     * @return
     */
    rclcpp::Node& getNode();

private:

    /**
     * @brief _reserved_color for the self collision
     */
    const color _reserved_color;

    XBot::ModelInterface::ConstPtr _model;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _collision_robot_pub;
    std::string _prefix;
    color _rgba;

    static Eigen::Affine3d toAffine3d(const urdf::Pose& p);

};

}


#endif // ROBOT_VIZ_H
