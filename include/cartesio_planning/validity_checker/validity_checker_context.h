#ifndef VALIDITY_CHECKER_LOADER_H
#define VALIDITY_CHECKER_LOADER_H


#include <iostream>
#include <yaml-cpp/yaml.h>

#include <cartesio_planning/validity_checker/planning_scene_wrapper.h>
#include <cartesio_planning/validity_checker/validity_predicate_aggregate.h>


namespace XBot { namespace Cartesian { namespace Planning {

class ValidityCheckContext
{

public:

    typedef std::unique_ptr<ValidityCheckContext> UniquePtr;
    typedef std::shared_ptr<ValidityCheckContext> Ptr;

    ValidityCheckContext();

    ValidityCheckContext(YAML::Node config,
                         ModelInterface::Ptr model, 
                         ros::NodeHandle nh = ros::NodeHandle());

    void setPlanningScene(PlanningSceneWrapper::Ptr ps);

    bool checkAll(bool verbose = false);

    PlanningSceneWrapper::Ptr planning_scene;
    ValidityPredicateAggregate vc_aggregate;

private:

    std::function<bool()> make_collision_checker(YAML::Node vc_node);

    ModelInterface::Ptr _model;
    
    ros::NodeHandle _nh;

};

} } }

#endif // VALIDITY_CHECKER_LOADER_H
