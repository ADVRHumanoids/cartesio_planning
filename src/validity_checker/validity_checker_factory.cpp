#include "validity_checker_factory.h"

#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/stability_detection.h"
#include "utils/parse_yaml_utils.h"

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

    // parse options
    YAML_PARSE_OPTION(vc_node, include_environment, bool, true);

    // construct planning scene for model
    auto planning_scene = std::make_shared<PlanningSceneWrapper>(model);
    planning_scene->startMonitor();

    // define validity checker
    auto validity_checker = [=]()
    {
        planning_scene->update();

        if(include_environment)
        {
            return !planning_scene->checkCollisions();
        }
        else
        {
            return !planning_scene->checkSelfCollisions();
        }

    };

    return validity_checker;

}

/**
 * @brief MakeConvexHullChecker
 * @param vc_node
 * @param model
 * @return
 */
std::function<bool ()> MakeConvexHullChecker(YAML::Node vc_node,
                                             XBot::ModelInterface::ConstPtr model,
                                             ros::NodeHandle& nh)
{
    using namespace XBot::Cartesian::Planning;

    YAML_PARSE_OPTION(vc_node, stability_margin, double, 0.0);
    YAML_PARSE_OPTION(vc_node, links, std::list<std::string>, {});

    auto cvx_hull = std::make_shared<ConvexHullStability>(model, links);
    auto cvx_ros = std::make_shared<ConvexHullROS>(model, *cvx_hull, nh);


    auto validity_checker = [=]()
    {
        cvx_ros->publish();
        return cvx_hull->checkStability();
    };

    return validity_checker;

}

}

std::function<bool ()> XBot::Cartesian::Planning::MakeValidityChecker(YAML::Node vc_node,
                                                                      ModelInterface::ConstPtr model,
                                                                      std::string lib_name,
                                                                      ros::NodeHandle& nh)
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
        if(vc_type == "CollisionCheck")
        {
            return MakeCollisionChecker(vc_node, model);
        }
        else if(vc_type == "ConvexHull")
        {
            return MakeConvexHullChecker(vc_node, model, nh);
        }
        else
        {
            throw std::runtime_error("Unsupported validity checker type '" + vc_type + "'");
        }
    }
}
