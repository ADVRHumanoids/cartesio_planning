#ifndef VALIDITY_CHECKER_LOADER_H
#define VALIDITY_CHECKER_LOADER_H


#include <iostream>
#include <yaml-cpp/yaml.h>
#include <thread>

#include "validity_checker/validity_predicate_aggregate.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/centroidal_statics.h"

#include <cartesio_planning/SetContactFrames.h>


namespace XBot { namespace Cartesian { namespace Planning {

class ValidityCheckContext
{

public:

    ValidityCheckContext();

    ValidityCheckContext(YAML::Node config,
                         ModelInterface::Ptr model, 
                         ros::NodeHandle nh = ros::NodeHandle(),
                         bool spin = false);
    
    static void spinOnce() 
    {
        while (ros::ok())
        {
            ros::Duration(0.01).sleep();
            ros::spinOnce();
        }
    };
    
    void sendContacts(std::vector<std::string> active_links, std::vector<std::vector<float>> rot);
    

    PlanningSceneWrapper::Ptr planning_scene;
    ValidityPredicateAggregate vc_aggregate;
    
private:

    std::function<bool()> make_collision_checker(YAML::Node vc_node);

    ModelInterface::Ptr _model;
    
    ros::NodeHandle _nh;
    
    ros::Publisher _pub;
    

};

} } }

#endif // VALIDITY_CHECKER_LOADER_H
