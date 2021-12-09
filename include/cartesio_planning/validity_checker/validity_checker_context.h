#ifndef VALIDITY_CHECKER_LOADER_H
#define VALIDITY_CHECKER_LOADER_H


#include <iostream>
#include <yaml-cpp/yaml.h>

#include "validity_checker/validity_predicate_aggregate.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/centroidal_statics.h"

namespace XBot { namespace Cartesian { namespace Planning {

class ValidityCheckContext
{

public:
    typedef std::unique_ptr<ValidityCheckContext> UniquePtr;

    ValidityCheckContext();

    ValidityCheckContext(YAML::Node config,
                         ModelInterface::Ptr model, 
                         ros::NodeHandle nh = ros::NodeHandle());

    void setPlanningScene(PlanningSceneWrapper::Ptr ps);

    PlanningSceneWrapper::Ptr planning_scene;
    ValidityPredicateAggregate vc_aggregate;

private:

    std::function<bool()> make_collision_checker(YAML::Node vc_node);

    ModelInterface::Ptr _model;
    
    ros::NodeHandle _nh;

};

} } }

#endif // VALIDITY_CHECKER_LOADER_H
