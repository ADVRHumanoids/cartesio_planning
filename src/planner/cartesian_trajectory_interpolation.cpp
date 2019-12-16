#include "planner/cartesian_trajectory_interpolation.h"
#include <RobotInterfaceROS/ConfigFromParam.h>

CartesianTrajectoryInterpolation::CartesianTrajectoryInterpolation()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    _model = XBot::ModelInterface::getModel(cfg);

    _interpolator = std::make_shared<TrajectoryInterpolation>(_model->getJointNum());
}

double CartesianTrajectoryInterpolation::compute(const std::vector<Eigen::VectorXd>& trajectory, std::vector<double> * time_point_vec)
{
    return _interpolator->compute(trajectory, time_point_vec);
}

Eigen::VectorXd CartesianTrajectoryInterpolation::evaluate(double t) const
{
    return _interpolator->evaluate(t);
}

void CartesianTrajectoryInterpolation::evaluate(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const
{
    _interpolator->evaluate(t, q, qdot);
}


bool CartesianTrajectoryInterpolation::isValid() const
{
    return _interpolator->isValid();
}

double CartesianTrajectoryInterpolation::getTrajectoryEndTime() const
{
    return _interpolator->getTrajectoryEndTime();
}

void CartesianTrajectoryInterpolation::setLimits(double qdot_max, double qddot_max)
{
    _interpolator->setLimits(qdot_max, qddot_max);
}

void CartesianTrajectoryInterpolation::setLimits(const Eigen::VectorXd& qdot_max, const Eigen::VectorXd& qddot_max)
{
    _interpolator->setLimits(qdot_max, qddot_max);
}

Eigen::Affine3d CartesianTrajectoryInterpolation::evaluate(double t, const std::string& base_link, const std::string& distal_link)
{
    Eigen::VectorXd q = _interpolator->evaluate(t);

    _model->setJointPosition(q);
    _model->update();

    Eigen::Affine3d pose;
    if(base_link == "world")
        _model->getPose(distal_link, pose);
    else
        _model->getPose(distal_link, base_link, pose);

    return pose;
}

void CartesianTrajectoryInterpolation::evaluate(double t, const std::string& base_link, const std::string& distal_link,
                                                Eigen::Affine3d& pose, Eigen::Vector6d& twist)
{
    Eigen::VectorXd q, qdot;
    _interpolator->evaluate(t, q, qdot);

    _model->setJointPosition(q);
    _model->setJointVelocity(qdot);
    _model->update();

    if(base_link == "world")
    {
        _model->getPose(distal_link, pose);
        _model->getVelocityTwist(distal_link, twist);
    }
    else
    {
        _model->getPose(distal_link, base_link, pose);
        _model->getVelocityTwist(distal_link, base_link, twist);
    }
}
