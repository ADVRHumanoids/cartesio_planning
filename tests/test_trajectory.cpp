#include <gtest/gtest.h>

#include <ReflexxesTypeII/Wrappers/TrajectoryGenerator.h>
#include <matlogger2/matlogger2.h>

#include "planner/trajectory_interpolation.h"

class TestTrajectory: public ::testing::Test {

protected:

    TestTrajectory()
    {
    }

    virtual ~TestTrajectory() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

};

//using namespace  XBot::Cartesian::Planning;

TEST_F(TestTrajectory, testTrajClass)
{
    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e5;
    auto logger = XBot::MatLogger2::MakeLogger("/tmp/testInterpolation3d_log", opt);

    TrajectoryInterpolation trj(34);

    const int n_points = 42;
    std::vector<Eigen::VectorXd> points(n_points);

    for(int i = 0; i < n_points; i++)
    {
        points[i].setRandom(34);
//        points[i] << sin(i/10.), 2*sin(i/10.), -3*sin(i/10.);
        logger->add("x_i", points[i]);
    }

    std::vector<double> times;
    double t_final = trj.compute(points, &times);

    logger->add("t_i", times);

    Eigen::Vector3d q = points[0];
    double t = 0;
    while(t < t_final)
    {
        q = trj.evaluate(t);
        t += 0.01;
        logger->add("x_traj", q);
        logger->add("t", t);
    }
}

TEST_F(TestTrajectory, testInterpolation3d)
{
    return;

    XBot::MatLogger2::Options opt;
    opt.default_buffer_size = 1e5;
    auto logger = XBot::MatLogger2::MakeLogger("/tmp/testInterpolation3d_log", opt);

    Eigen::Vector3d x, x0;
    x.setZero();
    x0.setZero();

    const double period = 0.01;

    Reflexxes::Utils::TrajectoryGenerator tg(x0.size(), period, x0);
    const double vmax = 1.0;
    tg.setVelocityLimits(vmax);
    tg.setAccelerationLimits(0.2);

    const int n_points = 100;
    std::vector<Eigen::Vector3d> points(n_points);

    for(int i = 0; i < n_points; i++)
    {
        points[i] << std::sin(i/10.), -2*std::sin(i/10.), 3*std::sin(i/10.);
    }

    int current_i = 0;
    double time = 0;
    Eigen::Vector3d v, vref;
    v.setZero();
    while( (x - points.back()).norm() > 0.001 )
    {
        vref.setZero();

        if(current_i-1 >= 0 && current_i+1 < n_points)
        {
            auto d_prev = (points[current_i] - points[current_i-1]);
            auto d_succ = (points[current_i+1] - points[current_i]);
            auto s_prev = d_prev.cwiseSign().cast<int>();
            auto s_succ = d_succ.cwiseSign().cast<int>();
            auto d_abs = (points[current_i+1] - points[current_i-1]).cwiseAbs();

            for(int j = 0; j < 3; j++)
            {
                if(s_prev(j) == s_succ(j))
                {
                    const double alpha = 1.5;
                    vref(j) = s_prev(j) * std::min(vmax, d_abs(j)*alpha);
                }
                else {
                    vref(j) = 0;
                }
            }
        }

        tg.setReference(points[current_i], vref);
        bool point_reached = tg.update(x, v);

        if(point_reached)
        {
            logger->add("t_p", time);
            logger->add("x_p", points[current_i]);
            current_i = std::min(current_i + 1, n_points - 1);
        }

        logger->add("x_traj", x);
        logger->add("v_traj", v);
        logger->add("t", time);

        time += period;

    }




};


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
