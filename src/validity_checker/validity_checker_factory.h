#ifndef VALIDITY_CHECKER_FACTORY_H
#define VALIDITY_CHECKER_FACTORY_H

#include <functional>
#include <XBotInterface/ModelInterface.h>
#include <ros/ros.h>

namespace XBot { namespace Cartesian { namespace Planning {

/**
     * @brief Factory function for validity checkers, which are returned as function objects.
     * @param task_node: yaml node defining the validity checker properties (must have a 'type' field)
     * @param model: ModelInterface that is used by the validity checker to do computations
     * @param lib_name: if not empty, name of the shared library containing the factory (e.g. libxxx.so)
     * It must be inside LD_LIBRARY_PATH.
     * @param nh: NodeHandle to be used for internal topics.
     * @return The validity checker
     */
    std::function<bool(void)> MakeValidityChecker(YAML::Node task_node,
                                                  ModelInterface::ConstPtr model,
                                                  std::string lib_name,
                                                  ros::NodeHandle& nh);

} } }

#endif // VALIDITY_CHECKER_FACTORY_H
