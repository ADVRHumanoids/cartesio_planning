#ifndef CARTESIO_PLANNING_H
#define CARTESIO_PLANNING_H

#include <cartesio_planning/state_space.h>
#include <yaml-cpp/yaml.h>

namespace XBot::Cartesian::Planning
{

class Planner
{

public:

    Planner(StateSpace::ConstPtr space,
            YAML::Node options);

    bool solve(Eigen::VectorXd qstart,
               Eigen::VectorXd qgoal,
               double timeout,
               std::string planner_type);

    ~Planner();


private:

    class Impl;

    std::unique_ptr<Impl> impl;

};

}

#endif // CARTESIO_PLANNING_H
