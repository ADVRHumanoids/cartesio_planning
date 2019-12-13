#ifndef TRAJECTORY_INTERPOLATION_H
#define TRAJECTORY_INTERPOLATION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <memory>

/**
 * @brief The TrajectoryInterpolation class computes a minimum time
 * trajectory passing through a specified list of waypoints, under
 * velocity and acceleration limits.
 *
 * Usage example (6-dof arm):
 * ^^^^^^^^^^^^^^^^^^^^^^^^^
 *
 * TrajectoryInterpolation trj(6);
 *
 * const double qdot_max  = 1.0;
 * const double qddot_max = 3.0;
 * trj.setLimits(qdot_max, qddot_max);
 *
 * std::vector<Eigen::VectorXd> points;
 * // fill points...
 *
 * double end_time = trj.compute(points);
 *
 * double dt = 0.01; // 100 Hz playback
 * for(double t = 0; t < end_time; t += dt)
 * {
 *     auto qi = trj.evaluate(t);
 *     // use qi...
 * }
 *
 */
class TrajectoryInterpolation
{

public:
    typedef std::shared_ptr<TrajectoryInterpolation> Ptr;

    TrajectoryInterpolation(int q_size);

    double compute(const std::vector<Eigen::VectorXd>& trajectory,
                   std::vector<double> * time_point_vec = nullptr);

    Eigen::VectorXd evaluate(double t) const;
    void evaluate(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const;

    bool isValid() const;

    double getTrajectoryEndTime() const;

    void setLimits(double qdot_max, double qddot_max);

    void setLimits(const Eigen::VectorXd& qdot_max,
                   const Eigen::VectorXd& qddot_max);

private:

    const int q_size;

    struct Poly
    {
        Eigen::VectorXd a_0, a_1, a_2, a_3;
        double t_start, t_end;

        Eigen::VectorXd eval(double abs_t) const;
        void eval(double abs_t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const;
    };

    std::vector<Poly> _spline;
    Eigen::ArrayXd _qdot_max, _qddot_max;


};

#endif // TRAJECTORY_INTERPOLATION_H
