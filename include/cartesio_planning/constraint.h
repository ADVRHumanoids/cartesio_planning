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

    Constraint(std::shared_ptr<const StateSpace> space,
               YAML::Node options = YAML::Node());

    void bind(std::shared_ptr<const StateSpace> space);

    void reset();

    int constraintSize() const;

    Eigen::VectorXd value(const Eigen::VectorXd& q) const;

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;

    virtual bool project(Eigen::VectorXd& q) const;

    virtual bool refine(Eigen::VectorXd& q) const;

    bool checkJacobian(const Eigen::VectorXd& q) const;

    Eigen::VectorXd sample() const;

    virtual ~Constraint();

protected:

    std::shared_ptr<const StateSpace> _space;

    class Impl;

    std::unique_ptr<Impl> impl;

    virtual Eigen::VectorXd _value(const Eigen::VectorXd& q) const = 0;

    virtual Eigen::MatrixXd _jacobian(const Eigen::VectorXd& q) const = 0;

    virtual int _constraintSize() const = 0;

public:

    Impl& getImpl() const;
};

}

#endif // CONSTRAINT_H
