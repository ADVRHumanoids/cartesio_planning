#ifndef CARTESIO_PLANNING_CONSTRAINT_H
#define CARTESIO_PLANNING_CONSTRAINT_H

#include <yaml-cpp/yaml.h>

#include "common/types.h"

namespace XBot::Cartesian::Planning
{
class StateSpace;

class Constraint : public std::enable_shared_from_this<Constraint>
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(Constraint);

    Constraint(YAML::Node options = YAML::Node());

    void bind(std::shared_ptr<const StateSpace> space);

    void reset();

    virtual int constraintSize() const = 0;

    virtual Eigen::VectorXd value(const Eigen::VectorXd& q) const = 0;

    virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const = 0;

    virtual bool project(Eigen::VectorXd& q) const;

    virtual bool refine(Eigen::VectorXd& q) const;

    bool checkJacobian(const Eigen::VectorXd& q) const;

    Eigen::VectorXd sample() const;

    virtual ~Constraint();

protected:

    std::shared_ptr<const StateSpace> _space;

    class Impl;

    std::unique_ptr<Impl> impl;

public:

    Impl& getImpl() const;
};
}

#endif // CONSTRAINT_H
