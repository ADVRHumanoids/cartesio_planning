#include "trajectory_interpolation.h"
#include <casadi/casadi.hpp>

namespace
{
/**
     * @brief operator << to insert an Eigen vector into a std vector
     */
    void operator<<(std::vector<double>& v, Eigen::VectorXd vi)
    {
        v.insert(v.end(), vi.data(), vi.data() + vi.size());
    }

}

/**
 * Utility to print a vector to a given output stream
 */
template <typename T>
std::ostream& print(std::ostream& os, const std::vector<T>& v)
{
    for(const auto& elem : v)
    {
        os << " -- " << elem << "\n";
    }

    return os;
}


TrajectoryInterpolation::TrajectoryInterpolation(int _q_size):
    q_size(_q_size)
{
    _qdot_max.setConstant(q_size, 1.0);
    _qddot_max.setConstant(q_size, 5.0);
}

double TrajectoryInterpolation::compute(const std::vector<Eigen::VectorXd>& trajectory,
                                      std::vector<double> * time_point_vec)
{
    const int n_points = trajectory.size();
    const int n_segments = n_points - 1;
    const int q_size = trajectory.back().size();

    Eigen::VectorXd a_min, a_max;
    a_max.setConstant(q_size, 100.);
    a_min = -a_max;

    Eigen::VectorXd qdot_min = -_qdot_max;
    Eigen::VectorXd qdot_init = _qdot_max * 0;

    Eigen::VectorXd qddot_max = _qddot_max;
    Eigen::VectorXd qddot_min = -qddot_max;
    Eigen::VectorXd qddot_init; qddot_init.setZero(q_size);

    std::vector<double> T_min(n_segments), T_max(n_segments, 100.);
    for(int i = 0; i < n_segments; i++)
    {
        T_min[i] = ((trajectory[i+1] - trajectory[i]).array().abs()/_qdot_max).maxCoeff();
        std::cout << "T_min[" << i << "] = " << T_min[i] << std::endl;
    }

    auto optvar = casadi::MX::sym("V", n_segments * (1 + 3 * q_size)); // a_1, a_2, a_3, T
    std::vector<casadi::MX> a_1, a_2, a_3, T;
    std::vector<double> v_min, v_max, v_init;

    // fill bounds and initial value
    int v_idx = 0;
    for(int i = 0; i < n_segments; i++)
    {
        // save a_1 variable (initial velocity)
        a_1.push_back(optvar.nz(casadi::Slice(v_idx, v_idx + q_size)));
        v_idx += q_size;

        // bounds on a_1
        if(i == 0) // initial velocity is zero
        {
            v_min.insert(v_min.end(), q_size, 0.0);
            v_max.insert(v_max.end(), q_size, 0.0);
            v_init.insert(v_init.end(), q_size, 0.0);
        }
        else // bounds on velocity
        {
            v_min << qdot_min;
            v_max << _qdot_max;
            v_init << qdot_init;
        }

        // save a_2 variable (twice the initial acceleration)
        a_2.push_back(optvar.nz(casadi::Slice(v_idx, v_idx + q_size)));
        v_idx += q_size;

        // bounds on a_2
        if(i == 0) // initial acceleration is zero
        {
            v_min.insert(v_min.end(), q_size, 0.0);
            v_max.insert(v_max.end(), q_size, 0.0);
            v_init.insert(v_init.end(), q_size, 0.0);
        }
        else // bounds on acceleration
        {
            v_min << qddot_min / 2.0;
            v_max << qddot_max / 2.0;
            v_init.insert(v_init.end(), q_size, 0.0);
        }

        // save a_3 variable (trajectory segment jerk) and push bounds
        a_3.push_back(optvar.nz(casadi::Slice(v_idx, v_idx + q_size)));
        v_idx += q_size;
        v_min << a_min;
        v_max << a_max;
        v_init.insert(v_init.end(), q_size, 0.0);

        // save T variable (duration of i-th segment) and push bounds
        T.push_back(optvar.nz(v_idx));
        v_idx += 1;
        v_min.insert(v_min.end(), T_min[i]);
        v_max.insert(v_max.end(), T_max[i]);
        v_init.insert(v_init.end(), T_min[i]*2.0);
    }

    // constraints
    std::vector<casadi::MX> g;
    std::vector<double> g_min, g_max;
    for(int i = 0; i < n_segments; i++)
    {
        // position continuity -> p_0 + a1*t + a2*t^2 + a3*t^3 == next p_0
        g.push_back(a_1[i]*T[i] + a_2[i]*T[i]*T[i] + a_3[i]*T[i]*T[i]*T[i]);
        g_min << trajectory[i+1] - trajectory[i];
        g_max << trajectory[i+1] - trajectory[i];

        if(i == n_segments - 1)
        {
            // zero final velocity
            g.push_back(a_1[i] + 2*a_2[i]*T[i] + 3*a_3[i]*T[i]*T[i]);
            g_min.insert(g_min.end(), q_size, 0);
            g_max.insert(g_max.end(), q_size, 0);
        }
        else
        {
            // velocity continuity -> a1 + 2*a2*t + 3*a3*t^2 == next a1
            g.push_back(a_1[i] + 2*a_2[i]*T[i] + 3*a_3[i]*T[i]*T[i] - a_1[i+1]);
            g_min.insert(g_min.end(), q_size, 0);
            g_max.insert(g_max.end(), q_size, 0);
        }

        if(i == n_segments - 1)
        {
            // zero final acceleration
            g.push_back(a_2[i] + 3*a_3[i]*T[i]);
            g_min.insert(g_min.end(), q_size, 0);
            g_max.insert(g_max.end(), q_size, 0);
        }
    }

    // cost function
    casadi::MX J = 0;
    for(int i = 0; i < n_segments; i++)
    {
        J += T[i];
        J += 1e-2 * casadi::MX::dot(a_1[i], a_1[i]);
        J += 1e-2 * casadi::MX::dot(a_2[i], a_2[i]);
        J += 1e-2 * casadi::MX::dot(a_3[i], a_3[i]);
    }

    // NLP
    casadi::MXDict nlp = {{"x", optvar}, {"f", J}, {"g", casadi::MX::vertcat(g)}};

    // Set options
    casadi::Dict opts;
    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.max_iter"] = 1000;
    opts["ipopt.linear_solver"] = "ma57";
    opts["ipopt.hessian_approximation"] = "limited-memory";

    // Create an NLP solver and buffers
    auto solver = nlpsol("nlpsol", "ipopt", nlp, opts);
    std::map<std::string, casadi::DM> arg, res;

    // Bounds and initial guess
    arg["lbx"] = v_min;
    arg["ubx"] = v_max;
    arg["lbg"] = g_min;
    arg["ubg"] = g_max;
    arg["x0"] = v_init;

    // Solve the problem (tbd: error checking)
    res = solver(arg);

    // Extract result
    casadi::Function a_1_fun("a_1_fun", {optvar}, a_1);
    casadi::Function a_2_fun("a_2_fun", {optvar}, a_2);
    casadi::Function a_3_fun("a_3_fun", {optvar}, a_3);
    casadi::Function T_fun("T_fun", {optvar}, T);

    auto a_1_dm = a_1_fun({res.at("x")});
    auto a_2_dm = a_2_fun({res.at("x")});
    auto a_3_dm = a_3_fun({res.at("x")});
    auto T_dm = T_fun({res.at("x")});

    // Contruct spline
    _spline.clear();

    if(time_point_vec)
    {
        time_point_vec->clear();
        time_point_vec->push_back(0);
    }

    for(int i = 0; i < n_segments; i++)
    {
        Poly p;
        p.a_0 = trajectory[i];
        p.a_1 = Eigen::VectorXd::Map(a_1_dm[i].ptr(), q_size);
        p.a_2 = Eigen::VectorXd::Map(a_2_dm[i].ptr(), q_size);
        p.a_3 = Eigen::VectorXd::Map(a_3_dm[i].ptr(), q_size);

        if(_spline.empty())
        {
            p.t_start = 0.0;
        }
        else
        {
            p.t_start = _spline.back().t_end;
        }

        p.t_end = p.t_start + T_dm[i].nonzeros().front();

        _spline.push_back(p);

        if(time_point_vec)
        {
            time_point_vec->push_back(p.t_end);
        }
    }

    return _spline.back().t_end;

}

Eigen::VectorXd TrajectoryInterpolation::evaluate(double t) const
{
    if(t < 0)
    {
        t = 0;
    }

    if(t > _spline.back().t_end)
    {
        t = _spline.back().t_end;
    }

    for(int i = 0; i < _spline.size(); i++)
    {
        if(t >= _spline[i].t_start && t <= _spline[i].t_end)
        {
            return _spline[i].eval(t);
        }
    }

    throw std::runtime_error("uh oh!");
}

void TrajectoryInterpolation::evaluate(double t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const
{
    if(t < 0)
    {
        t = 0;
    }

    if(t > _spline.back().t_end)
    {
        t = _spline.back().t_end;
    }

    for(int i = 0; i < _spline.size(); i++)
    {
        if(t >= _spline[i].t_start && t <= _spline[i].t_end)
        {
            _spline[i].eval(t, q, qdot);
        }
    }

    throw std::runtime_error("uh oh!");
}

bool TrajectoryInterpolation::isValid() const
{
    return !_spline.empty();
}

double TrajectoryInterpolation::getTrajectoryEndTime() const
{
    return isValid() ? _spline.back().t_end : -1.0;
}

void TrajectoryInterpolation::setLimits(double qdot_max, double qddot_max)
{
    _qdot_max.setConstant(q_size, qdot_max);
    _qddot_max.setConstant(q_size, qddot_max);
}

void TrajectoryInterpolation::setLimits(const Eigen::VectorXd & qdot_max,
                                        const Eigen::VectorXd & qddot_max)
{
    _qdot_max = qdot_max;
    _qddot_max = qddot_max;
}



Eigen::VectorXd TrajectoryInterpolation::Poly::eval(double abs_t) const
{
    if(abs_t < t_start) throw std::invalid_argument("Invalid time: before start");
    if(abs_t > t_end) throw std::invalid_argument("Invalid time: after end   ");

    double t = abs_t - t_start;

    Eigen::VectorXd q = a_0;
    q += a_1 * t;
    q += a_2 * t*t;
    q += a_3 * t*t*t;

    return q;
}

void TrajectoryInterpolation::Poly::eval(double abs_t, Eigen::VectorXd& q, Eigen::VectorXd& qdot) const
{
    q = eval(abs_t);

    double t = abs_t - t_start;

    qdot = a_1;
    qdot += 2.*a_2 * t;
    qdot += 3.*a_3 * t*t;
}
