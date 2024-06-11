#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <cartesio_planning/common/types.h>

#include <xbot2_interface/xbotinterface2.h>

#include <xbot2_interface/collision.h>

#include "constraint.h"


namespace XBot::Cartesian::Planning {

class StateValidityChecker;

class StateSpace {

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(StateSpace)

    enum Type {
        EUCLIDEAN,
        SO2,
        SE3,
        SE2,
        ROBOT_CONFIGURATION
    };

    struct RobotConfigurationSpaceOptions
    {
        Collision::CollisionModel::Ptr collision_model;
        Collision::CollisionModel::ComputeCollisionFreeOptions compute_coll_free_opt;
        bool sample_collision_free;

        RobotConfigurationSpaceOptions();
    };

    StateSpace();

    int addRobotConfigurationSpace(ModelInterface::Ptr model,
                                   RobotConfigurationSpaceOptions opt = RobotConfigurationSpaceOptions());

    int addSO3(Eigen::Vector3d qmin, Eigen::Vector3d qmax, std::string id = "");

    int addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id = "");

    int addSO2(double qmin, double qmax, std::string id = "");

    int addEuclidean(double qmin, double qmax, std::string id = "");

    int addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id = "");

    void setBounds(int i, Eigen::VectorXd qmin, Eigen::VectorXd qmax);

    void setBounds(std::string id, Eigen::VectorXd qmin, Eigen::VectorXd qmax);

    void setConstraint(Constraint::Ptr c);

    Eigen::VectorXd random() const;

    Eigen::VectorXd ambientRandom() const;

    int getNq() const;

    int getNv() const;

    int getNq(int i) const;

    int getQIndex(int i) const;

    ModelInterface::Ptr getModel(int i) const;

    Eigen::VectorXd sum(const Eigen::VectorXd& q1, const Eigen::VectorXd& v) const;

    Eigen::VectorXd interpolate(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, double tau) const;

    Eigen::VectorXd difference(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const;

    bool addStateValidityChecker(std::shared_ptr<const StateValidityChecker> svc);

    bool checkValid(const Eigen::VectorXd& q,
                    std::vector<std::string> * failed_checks = nullptr,
                    std::ostream& report_os = std::cerr) const;

    ~StateSpace();

private:

    class Impl;

    std::unique_ptr<Impl> impl;

public:

    Impl& getImpl() const;
    Impl& getImpl();

};

}

#endif // STATE_SPACE_H
