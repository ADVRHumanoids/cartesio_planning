#include "validity_checker_factory.h"

#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/stability_detection.h"
#include "validity_checker/stability/centroidal_statics.h"
#include "validity_checker/collisions/ground_collision.h"
#include "utils/parse_yaml_utils.h"

#include <boost/math/constants/constants.hpp>

namespace
{



/**
 * @brief MakeCollisionChecker
 * @param vc_node
 * @param model
 * @return
 */
std::function<bool ()> MakeCollisionChecker(YAML::Node vc_node,
                                            XBot::ModelInterface::Ptr model)
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
 * @param nh
 * @return validity_checker function
 */
std::function<bool ()> MakeCentroidalStaticsChecker(YAML::Node vc_node,
                                                    XBot::ModelInterface::Ptr model,
                                                    ros::NodeHandle nh)
{
    using namespace XBot::Cartesian::Planning;

    YAML_PARSE_OPTION(vc_node, eps, double, 1e-3);
    YAML_PARSE_OPTION(vc_node, links, std::vector<std::string>, {});
    YAML_PARSE_OPTION(vc_node, friction_coefficient, double, 0.5);
    YAML_PARSE_OPTION(vc_node, optimize_torque, bool, true);
    YAML_PARSE_OPTION(vc_node, rotations, std::vector<std::vector<double>>, {});
    YAML_PARSE_OPTION(vc_node, x_lim_cop, std::vector<double>, {});
    YAML_PARSE_OPTION(vc_node, y_lim_cop, std::vector<double>, {});

    Eigen::Vector2d x_lim_cop_eig, y_lim_cop_eig;
    x_lim_cop_eig.setZero();
    y_lim_cop_eig.setZero();
    if(x_lim_cop.size() > 0){
        x_lim_cop_eig[0] = x_lim_cop[0];
        x_lim_cop_eig[1] = x_lim_cop[1];}
    if(y_lim_cop.size() > 0){
        y_lim_cop_eig[0] = y_lim_cop[0];
        y_lim_cop_eig[1] = y_lim_cop[1];}

    auto cs = std::make_shared<CentroidalStatics>(model, links, friction_coefficient,
                                                  optimize_torque, x_lim_cop_eig, y_lim_cop_eig, false);

    // set rotations
    for (int i = 0; i < rotations.size(); i++)
    {
        Eigen::Quaternion<double> quat(rotations[i][3], rotations[i][0], rotations[i][1], rotations[i][2]);
        cs->setContactRotationMatrix(links[i], quat.toRotationMatrix());
    }

    auto cs_ros = std::make_shared<CentroidalStaticsROS>(model, cs, nh, eps);

    double ros_eps = cs_ros->getEps();

    if(std::sqrt(std::pow(eps-ros_eps,2)) > 1e-6)
        ROS_WARN("Centroidal Statics eps parameters have been set different in config and launch file!");

    auto validity_checker = [=]()
    {
        cs_ros->publish();
        return cs->checkStability(eps);
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
                                             XBot::ModelInterface::Ptr model,
                                             ros::NodeHandle nh)
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

std::function<bool ()> MakeDistanceCheck_tripod(YAML::Node planner_config,
                                                YAML::Node vc_node,
                                                XBot::ModelInterface::Ptr model)
{
    using namespace XBot::Cartesian::Planning;
    
    YAML_PARSE_OPTION(vc_node, min_x_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, min_y_distance, double, 0.0);
    YAML_PARSE_OPTION(planner_config["state_space"], ee_number, int, 2);
    YAML_PARSE_OPTION(planner_config["state_space"], end_effector, std::vector<std::string>, {});
    
    auto validity_checker = [=]()
    {
        std::vector<Eigen::Vector2d> ee(ee_number);
        Eigen::Affine3d T;

        for (int i = 0; i < ee_number; i++)
        {
            model->getPose(end_effector[i], T);
            ee[i] << T.translation().x(), T.translation().y();
        }
        
        for (int i = 0; i < ee_number-1; i++)
        {
            for (int j = i+1; j < ee_number; j++)
            {
                double x_distance = sqrt((ee[i](0) - ee[j](0)) * (ee[i](0) - ee[j](0)));
                double y_distance = sqrt((ee[i](1) - ee[j](1)) * (ee[i](1) - ee[j](1)));
                                
                if (x_distance < min_x_distance && y_distance < min_y_distance)
                {
                    return false;
                }
            }
        }
        
        return true;
    };
    
    return validity_checker;
    
}

std::function<bool ()> MakeGroundCollisionAvoidance(YAML::Node vc_node, 
                                                    XBot::ModelInterface::Ptr model,
                                                    ros::NodeHandle nh)
{
    YAML_PARSE_OPTION(vc_node, link, std::string, "");
    YAML_PARSE_OPTION(vc_node, axis, std::vector<double>, {});

    auto gc = std::make_shared<XBot::Cartesian::Planning::GroundCollision>(model);
    auto gc_ros = std::make_shared<XBot::Cartesian::Planning::GroundCollisionROS>(gc, model, nh);
    
    auto validity_checker = [gc_ros, gc]()
    {
        return gc->check();
    };
    
    return validity_checker;
}

}

std::function<bool ()> MakeDistanceCheck_centauro(YAML::Node planner_config,
                                                  YAML::Node vc_node,
                                                  XBot::ModelInterface::Ptr model)
{ 
    using namespace XBot::Cartesian::Planning;
    
    YAML_PARSE_OPTION(vc_node, max_x_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, max_y_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, front_rear_x_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, left_right_y_distance, double, 0.0);
    YAML_PARSE_OPTION(planner_config["state_space"], ee_number, int, 2);
    YAML_PARSE_OPTION(planner_config["state_space"], end_effector, std::vector<std::string>, {});
       
    auto validity_checker = [=]()
    {
        std::vector<Eigen::Vector2d> ee(ee_number);
        Eigen::Affine3d T;
        
        for (int i = 0; i < ee_number; i++)
        {
            model->getPose(end_effector[i], T);
            ee[i] << T.translation().x(), T.translation().y();
        }
        
        double x_diff; 
        double y_diff;                  
            
        for (int i = 0; i < ee_number; i += 2)
        {
            x_diff = sqrt((ee[i](0) - ee[i+1](0)) * (ee[i](0) - ee[i+1](0)));
            y_diff = sqrt((ee[i](1) - ee[i+1](1)) * (ee[i](1) - ee[i+1](1)));
            if (x_diff > max_x_distance || y_diff > max_y_distance)
            {
                return false;
            }
        }
            
        // Check feet crossing
        double xRel_w;
        double yRel_w;
            
        for (int i = 0; i < ee_number; i += 2)
        {
            xRel_w = ee[i](0) - ee[i+1](0);
            yRel_w = ee[i](1) - ee[i+1](1);
            if (yRel_w < 0.1)          
            {
                return false;        
            }
        }
         
        // Check distance between front and rear feet on x-axis
        const auto x_left = sqrt((ee[0](0) - ee[2](0)) * (ee[0](0) - ee[2](0)));
        const auto x_right = sqrt((ee[1](0) - ee[3](0)) * (ee[1](0) - ee[3](0)));
        const auto x_left_right = sqrt((ee[0](0) - ee[3](0)) * (ee[0](0) - ee[3](0)));
        const auto x_right_left = sqrt((ee[1](0) - ee[2](0)) * (ee[1](0) - ee[2](0)));
          
        if (x_right < front_rear_x_distance || x_left < front_rear_x_distance || x_right > 0.9 || x_left > 0.9 || x_left_right < front_rear_x_distance || x_right_left < front_rear_x_distance || x_right_left > 0.8 || x_left_right > 0.8)
        {
            return false;
        }
            
        // Check distance between front and rear feet on y-axis
        const auto y_front = sqrt((ee[0](1) - ee[1](1)) * (ee[0](1) - ee[1](1)));
        const auto y_rear = sqrt((ee[2](1) - ee[3](1)) * (ee[2](1) - ee[3](1)));
        const auto y_front_rear = sqrt((ee[0](1) - ee[3](1)) * (ee[0](1) - ee[3](1)));
        const auto y_rear_front = sqrt((ee[2](1) - ee[1](1)) * (ee[2](1) - ee[1](1)));        
            
        if (y_front < left_right_y_distance || y_rear < left_right_y_distance || y_front > 0.8 || y_rear > 0.8 || y_front_rear < left_right_y_distance || y_front_rear > 0.8 || y_rear_front < left_right_y_distance || y_rear_front > 0.8)
        {
            return false;
        }
        
        return true;
    };
    
    return validity_checker;
    
}

std::function<bool ()> MakeDistanceCheck_comanplus(YAML::Node planner_config,
                                               YAML::Node vc_node,
                                               XBot::ModelInterface::Ptr model)
{ 
    using namespace XBot::Cartesian::Planning;
    
    YAML_PARSE_OPTION(vc_node, max_x_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, max_y_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, min_x_distance, double, 0.0);
    YAML_PARSE_OPTION(vc_node, min_y_distance, double, 0.0);
    YAML_PARSE_OPTION(planner_config["state_space"], ee_number, int, 2);
    YAML_PARSE_OPTION(planner_config["state_space"], end_effector, std::vector<std::string>, {});
       
    auto validity_checker = [=]()
    {
        std::vector<Eigen::Vector3d> ee(ee_number);
        Eigen::Affine3d T;
        
        for (int i = 0; i < ee_number; i++)
        {
            model->getPose(end_effector[i], T);
            auto rot = acos(T.linear()(0,0));
            ee[i] << T.translation().x(), T.translation().y(), rot;
            
        }
        
        double x_diff = sqrt((ee[0](0) - ee[1](0)) * (ee[0](0) - ee[1](0))); 
        double y_diff = sqrt((ee[0](1) - ee[1](1)) * (ee[0](1) - ee[1](1)));

        if (x_diff > max_x_distance || y_diff > max_y_distance)
        {
            return false;
        }           
            
        // Check for relative orientation
        double res1 = (ee[0](2) - ee[1](2));
        double res2 = (boost::math::constants::pi<double>()*2 - (ee[0](2) - ee[1](2)));
        
//         if (std::min<double>(sqrt(res1*res1), sqrt(res2*res2)) > boost::math::constants::pi<double>()/6)
//         {
//             std::cout << ee[0](2) << "   " << ee[1](2) << std::endl;
//             std::cout << "res1: " << res1 << "       res2: " << res2 << std::endl;
//             std::cout << "relative orientation" << std::endl;
//             return false;
//         }
             
        // Check for feet crossing
        double xRel_w = ee[0](0) - ee[1](0);
        double yRel_w = ee[0](1) - ee[1](1);
//         std::cout << -xRel_w * sin(ee[1](2)) + yRel_w * cos(ee[1](2)) << std::endl;
    
        if (abs(-xRel_w * sin(ee[1](2)) + yRel_w * cos(ee[1](2))) < 0.15)
        {
            std::cout << "feet crossing" << std::endl;
            std::cout << -xRel_w * sin(ee[1](2)) + yRel_w * cos(ee[1](2)) << std::endl;
            return false;        
        }           
        
        return true;
    };
    
    return validity_checker;
}

std::function<bool ()> XBot::Cartesian::Planning::MakeValidityChecker(YAML::Node planner_config,
                                                                      YAML::Node vc_node,
                                                                      ModelInterface::Ptr model,
                                                                      std::string lib_name,
                                                                      ros::NodeHandle nh)
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
        else if(vc_type == "DistanceCheck_tripod")
        {
            return MakeDistanceCheck_tripod(planner_config, vc_node, model);
        }
        else if(vc_type == "DistanceCheck_centauro")
        {
            return MakeDistanceCheck_centauro(planner_config, vc_node, model);
        }
        else if(vc_type == "DistanceCheck_comanplus")
        {
            return MakeDistanceCheck_comanplus(planner_config, vc_node, model);
        }
        else if(vc_type == "CentroidalStatics")
        {
            return MakeCentroidalStaticsChecker(vc_node, model, nh);
        }
        else if(vc_type == "GroundCollisionAvoidance")
        {
            return MakeGroundCollisionAvoidance(vc_node, model, nh);
        }
        else
        {
            throw std::runtime_error("Unsupported validity checker type '" + vc_type + "'");
        }
    }
}
