#include "validity_checker_context.h"
#include "utils/parse_yaml_utils.h"
#include "validity_checker/validity_checker_factory.h"

namespace XBot { namespace Cartesian { namespace Planning {

ValidityCheckContext::ValidityCheckContext(YAML::Node config,
                                           ModelInterface::ConstPtr model,
                                           ros::NodeHandle& nh):
    _model( model)
{
    if(!config["state_validity_check"])
    {
        std::cout << "No state validity checkers were defined" << std::endl;
        return;
    }

    for(auto vc : config["state_validity_check"])
    {
        auto vc_name = vc.as<std::string>();

        if(!config[vc_name])
        {
            throw std::runtime_error("Node for state validity checker '" + vc_name + "' must be defined");
        }

        std::function<bool()> vc_fun;

        // handle collision avoidance separately, since it is a critical component
        if(config[vc_name]["type"].as<std::string>() == "CollisionCheck")
        {
            vc_fun = make_collision_checker(config[vc_name]); // this also create planning scene
        }
        else
        {
            vc_fun = MakeValidityChecker(config[vc_name],
                                         model,
                                         "", nh);
        }

        vc_aggregate.add(vc_fun, vc_name);

    }

}

std::function<bool ()> ValidityCheckContext::make_collision_checker(YAML::Node vc_node)
{
    using namespace XBot::Cartesian::Planning;

    // parse options
    YAML_PARSE_OPTION(vc_node, include_environment, bool, true);

    // construct planning scene for model
    planning_scene = std::make_shared<PlanningSceneWrapper>(_model);

    auto planning_scene_capture = planning_scene;

    // define validity checker
    auto validity_checker = [include_environment,
            planning_scene_capture]()
    {
        planning_scene_capture->update();

        if(include_environment)
        {
            return !planning_scene_capture->checkCollisions();
        }
        else
        {
            return !planning_scene_capture->checkSelfCollisions();
        }

    };

    return validity_checker;
}

XBot::Cartesian::Planning::ValidityCheckContext::ValidityCheckContext()
{

}

} } }
