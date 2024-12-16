#ifndef CONTACT_CONSTRAINT_H
#define CONTACT_CONSTRAINT_H

#include "../state_space.h"

namespace XBot::Cartesian::Planning
{

class ContactConstraint : public Constraint
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(ContactConstraint)

    ContactConstraint(std::shared_ptr<const StateSpace> space,
                      XBot::ModelInterface::Ptr model,
                      std::map<std::string, std::vector<int>> contact_map,
                      YAML::Node options = YAML::Node());

    int _constraintSize() const override;

    void resetContactPose();

    void setContactPose(std::string name,
                        Eigen::Affine3d T);

    void update(const Eigen::VectorXd &q) const;

    Eigen::VectorXd _value(const Eigen::VectorXd &q) const override;

    Eigen::MatrixXd _jacobian(const Eigen::VectorXd &q) const override;

    bool refine(Eigen::VectorXd& q) const override;

private:

    XBot::ModelInterface::Ptr _model;

    struct Contact {
        std::vector<int> indices;
        Eigen::Affine3d T;
    };

    std::map<std::string, Contact> _contact_map;

    mutable Eigen::VectorXd _val;

    mutable Eigen::MatrixXd _J;

    int _nc;

    mutable Eigen::VectorXd _old_q;

};
}

#endif // CONTACT_CONSTRAINT_H
