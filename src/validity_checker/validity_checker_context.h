#ifndef VALIDITY_CHECKER_LOADER_H
#define VALIDITY_CHECKER_LOADER_H


#include <iostream>
#include <yaml-cpp/yaml.h>

#include "validity_checker/validity_predicate_aggregate.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"

namespace XBot { namespace Cartesian { namespace Planning {

class ValidityCheckContext
{

public:

    ValidityCheckContext();

    ValidityCheckContext(YAML::Node config,
                         ModelInterface::ConstPtr model, ros::NodeHandle& nh);

    PlanningSceneWrapper::Ptr planning_scene;
    ValidityPredicateAggregate vc_aggregate;

private:

    std::function<bool()> make_collision_checker(YAML::Node vc_node);

    ModelInterface::ConstPtr _model;

};

} } }

#endif // VALIDITY_CHECKER_LOADER_H
