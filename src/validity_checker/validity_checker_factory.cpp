#include "validity_checker_factory.h"

#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/stability_detection.h"
#include "validity_checker/stability/centroidal_statics.h"
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
 * @brief MakeCentroidalStaticsChecker
 * @param vc_node
 * @param model
 * @return
 */
std::function<bool ()> MakeCentroidalStaticsChecker(YAML::Node vc_node,
                                                    XBot::ModelInterface::ConstPtr model,
                                                    ros::NodeHandle& nh)
{
    using namespace XBot::Cartesian::Planning;

    YAML_PARSE_OPTION(vc_node, stability_margin, double, 1e-3);
    YAML_PARSE_OPTION(vc_node, links, std::vector<std::string>, {});
    YAML_PARSE_OPTION(vc_node, friction_coefficient, double, 0.5);
    YAML_PARSE_OPTION(vc_node, optimize_torque, bool, false);
    YAML_PARSE_OPTION(vc_node, x_lims, std::vector<double>, {});
    YAML_PARSE_OPTION(vc_node, y_lims, std::vector<double>, {});

    Eigen::Vector2d xlims, ylims;
    if(optimize_torque)
    {
        if(x_lims.empty() || x_lims.size() > 2 || y_lims.empty() || y_lims.size() > 2)
            throw std::runtime_error("If optimize_torque is set to true, x_lims = [xl, xu] and y_lims = [yl, yu] parameters needs to be set! ");

        xlims[0] = x_lims[0];
        xlims[1] = x_lims[1];
        ylims[0] = y_lims[0];
        ylims[1] = y_lims[1];
    }

    std::shared_ptr<CentroidalStatics> cs;
    if(optimize_torque)
        cs = std::make_shared<CentroidalStatics>(model, links, friction_coefficient, optimize_torque, xlims, ylims);
    else
        cs = std::make_shared<CentroidalStatics>(model, links, friction_coefficient, optimize_torque);
    auto cs_ros = std::make_shared<CentroidalStaticsROS>(model, *cs, nh);

    auto validity_checker = [=]()
    {
        cs_ros->publish();
        return cs->checkStability(stability_margin);
        return true;
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
        else if(vc_type == "CentroidalStatics")
        {
            return MakeCentroidalStaticsChecker(vc_node, model, nh);
        }
        else
        {
            throw std::runtime_error("Unsupported validity checker type '" + vc_type + "'");
        }
    }
}
