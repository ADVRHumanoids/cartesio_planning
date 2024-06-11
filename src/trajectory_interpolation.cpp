#include <cartesio_planning/trajectory_interpolation.h>
#include <xbot2_interface/common/utils.h>
#include "ros/impl/utils.hxx"

namespace XBot::Cartesian::Planning {

trajectory_msgs::JointTrajectory simpleInterpolation(StateSpace& ss,
                                                     const Eigen::MatrixXd &wp,
                                                     const Eigen::VectorXd &max_vel,
                                                     const Eigen::VectorXd &max_acc,
                                                     double dt)
{
    const int n = wp.rows();

    // max vel at tau = 0.5 -> 1.875
    // max acc at tau = 0.211 -> 5.774

    trajectory_msgs::JointTrajectory trj;

    // first node
    trj.points.resize(1);
    utils::eigenToStd(wp.col(0), trj.points[0].positions);
    trj.points[0].velocities.assign(n, 0.);
    trj.points[0].accelerations.assign(n, 0.);
    trj.points[0].time_from_start.fromSec(0);

    // loop over segments
    for(int s = 0; s < wp.cols()-1; s++)
    {
        // vel = 1.875 / dur
        // acc = 5.774 / dur^2
        double segment_duration_velmax = (1.875 / max_vel.array()).maxCoeff();
        double segment_duration_accmax = std::sqrt((5.774 / max_acc.array()).maxCoeff());
        double segment_duration = std::max(segment_duration_velmax, segment_duration_accmax);

        // dt < 0 means don't interpolate, just compute node times
        if(dt < 0)
        {
            trajectory_msgs::JointTrajectoryPoint pt;
            utils::eigenToStd(wp.col(s+1), pt.positions);
            pt.velocities.assign(n, 0.);
            pt.accelerations.assign(n, 0.);
            pt.time_from_start.fromSec(trj.points.back().time_from_start.toSec() + segment_duration);
            trj.points.push_back(std::move(pt));
            continue;
        }

        // interpolate
        const int n_nodes = std::ceil(segment_duration / dt);
        const double segment_start_time = trj.points.back().time_from_start.toSec();
        const auto& qstart = wp.col(s);
        const auto& qend = wp.col(s+1);
        auto delta_q = ss.difference(qend, qstart);

        for(int i = 0; i < n_nodes + 1; i++)
        {
            double node_time = float(i) / n_nodes * segment_duration;
            double alpha = node_time / segment_duration;
            auto [tau, dtau, ddtau] = Utils::quinticSplineDerivatives(alpha);

            trajectory_msgs::JointTrajectoryPoint pt;
            utils::eigenToStd(ss.interpolate(qstart, qend, tau), pt.positions);
            utils::eigenToStd(dtau*delta_q/segment_duration, pt.velocities);
            utils::eigenToStd(ddtau*delta_q/segment_duration/segment_duration, pt.accelerations);
            pt.time_from_start.fromSec(segment_start_time + node_time);

            trj.points.push_back(std::move(pt));
        }
    }

    return trj;

}

} // namespace XBot::Cartesian::Planning
