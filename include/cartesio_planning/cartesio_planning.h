#ifndef CARTESIO_PLANNING_H
#define CARTESIO_PLANNING_H

#include <yaml-cpp/yaml.h>

#include "state_validity_checker.h"

namespace XBot::Cartesian::Planning
{

class Planner
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(Planner)

    Planner(StateSpace::ConstPtr space,
            YAML::Node options);

    bool addStateValidityChecker(StateValidityChecker::ConstPtr svc);

    bool checkValid(const Eigen::VectorXd& q,
                    std::vector<std::string> * failed_checks = nullptr,
                    std::ostream& report_os = std::cerr) const;

    bool solve(Eigen::VectorXd qstart,
               Eigen::VectorXd qgoal,
               double timeout,
               std::string planner_type);

    Eigen::MatrixXd getSolutionPath(bool simplify = false, double timeout = -1) const;

    ~Planner();


private:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

#endif // CARTESIO_PLANNING_H
