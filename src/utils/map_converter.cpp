#include <utils/map_converter.h>

using namespace XBot::Converter;

MapConverter::MapConverter(nav_msgs::OccupancyGrid map):
    _map(map)  
{}

MapConverter::MapConverter(ros::NodeHandle& nh, 
                           const std::string& topic_name):
    _nh(nh)
{
    _sub = _nh.subscribe<nav_msgs::OccupancyGrid>(topic_name, 10, &MapConverter::callback, this);
}

void MapConverter::callback(const nav_msgs::OccupancyGrid msg) 
{
    _map = msg;
}

void MapConverter::setMap(nav_msgs::OccupancyGrid map)
{
    _map = map;
}

nav_msgs::OccupancyGrid MapConverter::getMap() const
{
    return _map;
}

void MapConverter::convert () 
{
    for (int width = 0; width < _map.info.width; width++)
    {
        for (int height = 0; height < _map.info.height; height++)
        {
            if (_map.data[height*_map.info.width + width] > 0 || _map.data[height*_map.info.width + width] == -1)
            {
                _pos_occ.push_back({width * _map.info.resolution + _map.info.resolution/2 + _map.info.origin.position.x,
                                    height * _map.info.resolution + _map.info.resolution/2 + _map.info.origin.position.y});
            }
        }
    }    
}

void MapConverter::getOccupiedPoints (Eigen::VectorXd x, 
                                      Eigen::VectorXd y)
{
//     double* x_ptr = &_x_occ[0];
//     double* y_ptr = &_y_occ[0];
//     Eigen::VectorXd::Map(x_ptr, _x_occ.size()) = x;
//     Eigen::VectorXd::Map(y_ptr, _y_occ.size()) = y;
}


bool MapConverter::checkForCollision (Eigen::VectorXd pos, 
                                      double size) 
{
//     bool check = false;
    
    float res = _map.info.resolution;
    
    return std::any_of(_pos_occ.begin(), _pos_occ.end(), [pos, size, res](std::vector<double> i)
                       {return sqrt((pos(0) - i[0]) * (pos(0) - i[0])) < size/2 + res/2 && sqrt((pos(1) - i[1]) * (pos(1) - i[1])) < size/2 + res/2;});
             
    
//     for (int i = 0; i < _x_occ.size() ; i++)
//     {
//         if (sqrt((pos(0) - _x_occ[i]) * (pos(0) - _x_occ[i])) < size/2 + _map.info.resolution/2 &&
//             sqrt((pos(1) - _y_occ[i]) * (pos(1) - _y_occ[i])) < size/2 + _map.info.resolution/2)
//         {
//             check = true;
//         }
//     }
//     
//     return check;
}


