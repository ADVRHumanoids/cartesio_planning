#include "common.h"

#include <cartesio_planning/cartesio_planning.h>

#include <cartesio_planning/state_validity_checker/collision.h>

using namespace XBot::Cartesian::Planning;

using TestBasic = TestWithModel;

TEST_F(TestBasic, check1)
{
    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    Planner planner(space, YAML::Node());

    auto qstart = model->generateRandomQ();

    auto qgoal = model->getRobotState("home");

    EXPECT_TRUE(planner.solve(qstart, qgoal, 1.0, "RRTConnect"));

    auto trj = planner.getSolutionPath(true);

    EXPECT_EQ(trj.cols(), 2);

    EXPECT_EQ(trj.rows(), model->getNq());
}

TEST_F(TestBasic, checkStateValidityChecker)
{
    model->getJoint(0)->setJointLimits(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    Planner planner(space, YAML::Node());

    auto qstart = model->generateRandomQ();

    auto qgoal = model->getRobotState("home");

    // obstacle: forbid passing thorugh the midpoint between start and goal
    double dist = model->difference(qgoal, qstart).norm();

    Eigen::VectorXd midpoint = model->sum(qstart, model->difference(qgoal, qstart)/2.);

    auto svcfun = std::make_shared<StateValidityCheckerFunction>(
        space, "mysvc",
        [&](const Eigen::VectorXd& q)
        {
            return model->difference(q, midpoint).norm() >= dist/4;
        });

    planner.addStateValidityChecker(svcfun);

    // solve
    ASSERT_TRUE(planner.solve(qstart, qgoal, 1.0, "RRTConnect"));

    auto trj = planner.getSolutionPath(true);

    EXPECT_GT(trj.cols(), 2);

    EXPECT_EQ(trj.rows(), model->getNq());
}

TEST_F(TestBasic, checkCollisionVC)
{
    model->getJoint(0)->setJointLimits(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    Planner planner(space, YAML::Node());

    auto qstart = model->getRobotState("home");
    qstart[model->getQIndex("torso_yaw")] = 2;

    auto qgoal = model->getRobotState("home");
    qstart[model->getQIndex("torso_yaw")] = -2;

    auto T_base = model->getPose("base_link");
    Eigen::Affine3d T_obs;
    T_obs.setIdentity();
    T_obs.translation() << 1.0, 0.0, 0.0;


    auto coll = std::make_shared<XBot::Collision::CollisionModel>(model);

    coll->setLinkPairs({});

    XBot::Collision::Shape::Sphere obs;
    obs.radius = 0.4;
    coll->addCollisionShape("mysphere", "world", obs, T_base*T_obs);

    auto vc = std::make_shared<CollisionValidityChecker>(space, coll);

    planner.addStateValidityChecker(vc);

    ASSERT_TRUE(planner.solve(qstart, qgoal, 1.0, "RRTConnect"));

    std::cout << "num points = " << planner.getSolutionPath().cols() << "\n";


}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
