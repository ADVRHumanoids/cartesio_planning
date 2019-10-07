#ifndef VALIDITY_CHECKER_FACTORY_H
#define VALIDITY_CHECKER_FACTORY_H

#include <functional>
#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian { namespace Planning {

    std::function<bool(void)> MakeValidityChecker(YAML::Node task_node,
                                                  ModelInterface::ConstPtr model,
                                                  std::string name,
                                                  std::string lib_name);

} } }

#endif // VALIDITY_CHECKER_FACTORY_H
