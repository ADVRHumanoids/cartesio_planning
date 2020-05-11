#ifndef _MAP_CONVERTER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Dense>

namespace XBot { namespace Converter {
    
class MapConverter{

public:
    // Constructors
    MapConverter(nav_msgs::OccupancyGrid map);
    MapConverter(ros::NodeHandle& nh,
                 const std::string& topic_name);
    
    // Callback function for constructor 
    void callback(const nav_msgs::OccupancyGrid msg);
    
    // I/O
    void setMap(nav_msgs::OccupancyGrid map);
    nav_msgs::OccupancyGrid getMap() const;
    void getOccupiedPoints(Eigen::VectorXd x,
                           Eigen::VectorXd y);
    
    // This function extract x and y position of the centers of the occupied cells
    void convert();
    
    // Checks for collision looking whether the volume occupied by the foot is inside 
    // at least one cell volume. Returns true if collision is found
    bool checkForCollision(Eigen::VectorXd pos,
                           double size);
    
    ros::Subscriber _sub;

    
    
private:
    nav_msgs::OccupancyGrid _map;
    
    std::vector<double> _x_occ, _y_occ;
    
    ros::NodeHandle _nh;
};

} }

#endif
