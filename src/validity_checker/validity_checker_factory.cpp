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
// TODO: must be modified for a more general n-legs case
std::function<bool ()> MakeDistanceChecker(YAML::Node vc_node,
                                            XBot::ModelInterface::ConstPtr model)
{
    if (!vc_node["end_effector"])
    {
        throw std::runtime_error("Mandatory private 'end_effector' parameter missing");
    }
    YAML_PARSE_OPTION(vc_node, end_effector, std::vector<std::string>, {});
    
    if (!vc_node["max_x_distance"])
    {
        std::cout << "'max_x_distance' parameter missing, using default 0.5 value" << std::endl;
    }
    YAML_PARSE_OPTION(vc_node, max_x_distance, double, 0.5);
    
    if (!vc_node["max_y_distance"])
    {
        std::cout << "'max_y_distance' parameter missing, using default 0.35 value" << std::endl;
    }
    YAML_PARSE_OPTION(vc_node, max_y_distance, double, 0.35);
    
    auto validity_checker = [=] ()
    {        
        Eigen::Affine3d T; 
        std::vector<double> x (end_effector.size());
        std::vector<double> y (end_effector.size());
        for (int i = 0; i < end_effector.size(); i++)
        {
            model->getPose(end_effector[i], T);
            x[i] = T.translation().x();
            y[i] = T.translation().y();
        }
        
//         double difference_x = std::max_element(x.begin(), x.end()) - std::min_element(x.begin(),x.end());
//         double difference_y = std::max_element(y.begin(), y.end()) - std::min_element(y.begin(),y.end());

        double difference_x = abs(x[1] - x[0]);
        double difference_y = abs(y[1] - y[0]);
    
        if (difference_x < max_x_distance && difference_y < max_y_distance)
            return true;
        else
        {
            std::cout << "difference_x: " << difference_x << " ("<< x[0] << "-" << x[1] << ") " << "and difference_y: " << difference_y << " ("<< y[0] << "-" << y[1] << ") " << " violates bounds" << std::endl;
            return false;
        }
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
        else if(vc_type == "Distance")
        {
            return MakeDistanceChecker(vc_node, model);
        }
        else
        {
            throw std::runtime_error("Unsupported validity checker type '" + vc_type + "'");
        }
    }
}
