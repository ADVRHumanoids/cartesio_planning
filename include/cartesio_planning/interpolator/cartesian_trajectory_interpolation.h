#ifndef CARTESIAN_TRAJECTORY_INTERPOLATION_H
#define CARTESIAN_TRAJECTORY_INTERPOLATION_H

#include "planner/trajectory_interpolation.h"
#include <XBotInterface/ModelInterface.h>

class CartesianTrajectoryInterpolation
{
public:
    typedef std::shared_ptr<CartesianTrajectoryInterpolation> Ptr;

    CartesianTrajectoryInterpolation();

    double compute(const std::vector<Eigen::VectorXd>& trajectory,
                   std::vector<double> * time_point_vec = nullptr);

    Eigen::VectorXd evaluate(double t) const;
    void evaluate(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const;

    Eigen::Affine3d evaluate(double t, const std::string& base_link, const std::string& distal_link);
    void evaluate(double t, const std::string& base_link, const std::string& distal_link,
                  Eigen::Affine3d& pose, Eigen::Vector6d& twist);

    bool isValid() const;

    double getTrajectoryEndTime() const;

    void setLimits(double qdot_max, double qddot_max);

    void setLimits(const Eigen::VectorXd& qdot_max,
                   const Eigen::VectorXd& qddot_max);
private:
    XBot::ModelInterface::Ptr _model;
    TrajectoryInterpolation::Ptr _interpolator;
};

#endif // CARTESIAN_TRAJECTORY_INTERPOLATION_H
