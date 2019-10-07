#include "validity_checker_factory.h"

#include "validity_checker/collisions/planning_scene_wrapper.h"

namespace
{

/**
 * @brief MakeCollisionChecker
 * @param vc_node
 * @param model
 * @return
 */
std::function<bool ()> MakeCollisionChecker(YAML::Node vc_node,
                                            XBot::ModelInterface::ConstPtr model)
{
    using namespace XBot::Cartesian::Planning;

    /* Macro for option parsing */
#define PARSE_OPTION(yaml, name, type, default_value) \
    type name = default_value; \
    if(yaml[#name]) \
    { \
    type value = yaml[#name].as<type>(); \
    std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
    name = value; \
} \
    else { \
    std::cout << "No option " #name " specified" << std::endl; \
} \
    /* End macro for option parsing */

    // parse options
    PARSE_OPTION(vc_node, include_environment, bool, true);

    // construct planning scene for model
    auto planning_scene = std::make_shared<PlanningSceneWrapper>(model);
    planning_scene->startMonitor();

    // define validity checker
    auto validity_checker = [=]()
    {
        planning_scene->update();

        if(include_environment)
        {
            return planning_scene->checkCollisions();
        }
        else
        {
            return planning_scene->checkSelfCollisions();
        }

    };

    return validity_checker;

}

/**
 * @brief MakeCollisionChecker
 * @param vc_node
 * @param model
 * @return
 */
std::function<bool ()> MakeConvexHullChecker(YAML::Node vc_node,
                                             XBot::ModelInterface::ConstPtr model)
{
    using namespace XBot::Cartesian::Planning;

}

}

std::function<bool ()> XBot::Cartesian::Planning::MakeValidityChecker(YAML::Node vc_node,
                                                                      ModelInterface::ConstPtr model,
                                                                      std::string name,
                                                                      std::string lib_name)
{
    /* Obtain factory name from task type */
    std::string vc_type = vc_node["type"].as<std::string>();
    std::string factory_name = vc_type + "ValidityCheckerFactory";

    /* Load task descripton from library */
    if(!lib_name.empty())
    {
        throw std::runtime_error("Unsupported specifying lib name");
    }
    else
    {
        if(vc_type == "CollisionChecker")
        {
            return MakeCollisionChecker(vc_node, model);
        }
        else if(vc_type == "ConvexHull")
        {

        }
        else
        {
            throw std::runtime_error("Unsupported validity checker type '" + vc_type + "'");
        }
    }
}
