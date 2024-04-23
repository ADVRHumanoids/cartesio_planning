#ifndef STATE_SPACE_H
#define STATE_SPACE_H

#include <cartesio_planning/common/types.h>

#include <xbot2_interface/xbotinterface2.h>

namespace XBot::Cartesian::Planning {

class StateSpace {

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(StateSpace)

    enum Type {
        EUCLIDEAN,
        SO2,
        SE3,
        SE2
    };

    StateSpace();

    int addRobotConfigurationSpace(ModelInterface::ConstPtr model);

    int addSO3(Eigen::Vector3d qmin, Eigen::Vector3d qmax, std::string id = "");

    int addSE3(Eigen::Vector6d qmin, Eigen::Vector6d qmax, std::string id = "");

    int addSO2(double qmin, double qmax, std::string id = "");

    int addEuclidean(double qmin, double qmax, std::string id = "");

    int addEuclidean(Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string id = "");

    void setBounds(int i, Eigen::VectorXd qmin, Eigen::VectorXd qmax);

    void setBounds(std::string id, Eigen::VectorXd qmin, Eigen::VectorXd qmax);

    std::pair<Eigen::VectorXd, Eigen::VectorXd> getBounds() const;

    ~StateSpace();

private:

    class Impl;

    std::unique_ptr<Impl> impl;

public:

    Impl& getImpl();

};

}

#endif // STATE_SPACE_H
