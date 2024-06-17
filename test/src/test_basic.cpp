#include "common.h"

#include <cartesio_planning/cartesio_planning.h>

#include <cartesio_planning/state_validity_checker/collision.h>

#include <cartesio_planning/constraints/cartesian_constraint.h>

#include <cartesio_planning/constraints/contact_constraint.h>

#include <cartesio_planning/trajectory_interpolation.h>

#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "../src/impl/profiling.hxx"


using namespace XBot::Cartesian::Planning;

using TestBasic = TestWithModel;

TEST_F(TestBasic, check1)
{
    model->getJoint(0)->setJointLimits(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    Planner planner(space, YAML::Node());

    auto qstart = model->generateRandomQ();

    auto qgoal = model->getRobotState("home");

    // check we can just interpolate start and goal
    for(double tau = 0; tau <= 1.0; tau += 0.01)
    {
        auto qk = space->interpolate(qstart, qgoal, tau);
        EXPECT_TRUE(space->checkValid(qk));
    }

    EXPECT_TRUE(planner.solve(qstart, qgoal, 10.0, "RRTConnect"));

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

    space->addStateValidityChecker(svcfun);

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

    space->addStateValidityChecker(vc);

    ASSERT_TRUE(planner.solve(qstart, qgoal, 1.0, "RRTConnect"));

    std::cout << "num points = " << planner.getSolutionPath().cols() << "\n";
}

class SphereConstraint : public Constraint
{
    // QTC_TEMP
    // Constraint interface
public:
    using Constraint::Constraint;
    int constraintSize() const override;
    Eigen::VectorXd value(const Eigen::VectorXd &q) const override;
    Eigen::MatrixXd jacobian(const Eigen::VectorXd &q) const override;
};

int SphereConstraint::constraintSize() const { return 1; }

Eigen::VectorXd SphereConstraint::value(const Eigen::VectorXd &q) const
{
    Eigen::VectorXd ret(1);
    ret << q.squaredNorm() - 1;
    return ret;
}

Eigen::MatrixXd SphereConstraint::jacobian(const Eigen::VectorXd &q) const
{
    return 2*q.transpose();
}

TEST_F(TestBasic, checkConstraint)
{
    auto space = std::make_shared<StateSpace>();

    space->addEuclidean(-2.*Eigen::Vector3d::Ones(),
                        2.*Eigen::Vector3d::Ones(),
                        "3dspace");

    auto c = std::make_shared<SphereConstraint>();
    c->bind(space);

    EXPECT_TRUE(c->checkJacobian(Eigen::Vector3d::Random()));

    space->setConstraint(c);

    Planner planner(space, YAML::Node());

    bool ret = planner.solve(Eigen::Vector3d(1, 0, 0),
                             Eigen::Vector3d(0, 1, 0),
                             1.0,
                             "PRMstar");

    EXPECT_TRUE(ret);

    auto trj = planner.getSolutionPath();

    std::cout << "trj = \n" << trj << "\n";

    for(int i = 0; i < trj.cols(); i++)
    {
        EXPECT_LT(c->value(trj.col(i)).norm(), 1e-3);
    }

    auto trjinterp = simpleInterpolation(*space,
                        trj,
                        Eigen::Vector3d::Constant(0.1),
                        Eigen::Vector3d::Constant(0.1),
                        0.01);

    for(auto& pt : trjinterp.points)
    {
        auto q = Eigen::Vector3d::Map(pt.positions.data());

        EXPECT_LT(c->value(q).norm(), 1e-3);
    }
}

TEST_F(TestBasic, checkConstraintCartesian)
{
    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    std::vector<std::string> contacts = {
        "contact_1",
        "contact_2",
        "contact_3",
        "contact_4"
    };

    std::string ik_pb_txt = R"(

stack:
  - [c1, c2, c3, c4]

c1:
  type: Cartesian
  distal_link: contact_1
c2:
  type: Cartesian
  distal_link: contact_2
c3:
  type: Cartesian
  distal_link: contact_3
c4:
  type: Cartesian
  distal_link: contact_4


    )";

    auto ik_pb = YAML::Load(ik_pb_txt);

    auto params = std::make_shared<XBot::Cartesian::Parameters>(1.0);

    auto ctx = std::make_shared<XBot::Cartesian::Context>(params, model);

    XBot::Cartesian::ProblemDescription pb(ik_pb, ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    model->setJointPosition(model->getRobotState("home"));
    model->update();
    ci->reset(0);
    EXPECT_TRUE(ci->update(0, 0));

    auto constr = std::make_shared<CartesianConstraint>(ci);
    constr->bind(space);

    EXPECT_EQ(constr->constraintSize(), 6*4);

    space->setConstraint(constr);

    EXPECT_LT(constr->value(model->getJointPosition()).norm(), 1e-4);

    Eigen::VectorXd qstart = model->getJointPosition(),
                    qgoal = model->getJointPosition();

    qgoal[model->getQIndex("knee_pitch_1")] *= 0.2;
    qgoal[model->getQIndex("knee_pitch_2")] *= 0.2;
    qgoal[model->getQIndex("knee_pitch_3")] *= 0.2;
    qgoal[model->getQIndex("knee_pitch_4")] *= 0.2;

    EXPECT_GT(constr->value(qgoal).norm(), 1e-3);

    EXPECT_TRUE(constr->project(qgoal));

    EXPECT_LT(constr->value(qgoal).norm(), 1e-4);

    std::cout << "deltaq = " << model->difference(qgoal, qstart).norm() << "\n";

    YAML::Node planner_cfg;

    planner_cfg["Atlas"]["Rho"] = 2.0;
    planner_cfg["Atlas"]["Epsilon"] = 0.1;
    planner_cfg["Atlas"]["Alpha"] = M_PI/16.;
    planner_cfg["Atlas"]["Exploration"] = 0.8;

    Planner planner(space, planner_cfg);

    EXPECT_TRUE(planner.solve(qstart, qgoal, 1.0, "RRTConnect"));

    auto trj = planner.getSolutionPath();

    std::cout << "n_segments = " << trj.cols() << "\n";

    EXPECT_LT(model->difference(trj.col(trj.cols()-1), qgoal).norm(), 1e-3);

    auto trjinterp = simpleInterpolation(*space,
                        trj,
                        Eigen::VectorXd::Constant(model->getNv(), 1.0),
                        Eigen::VectorXd::Constant(model->getNv(), 2.0),
                        0.01);

    for(auto& p : trjinterp.points)
    {
        auto q = Eigen::VectorXd::Map(p.positions.data(),
                                      p.positions.size());

        EXPECT_LT(constr->value(q).norm(), 1e-3);
    }
}

TEST_F(TestBasic, checkSampleConstraint)
{
    model->getJoint(0)->setJointLimits(-Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6));

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    std::vector<std::string> contacts = {
        "contact_1",
        "contact_2",
        "contact_3",
        "contact_4"
    };

    std::string ik_pb_txt = R"(

stack:
  - [c1, c2, c3, c4]

c1:
  type: Cartesian
  distal_link: contact_1
c2:
  type: Cartesian
  distal_link: contact_2
c3:
  type: Cartesian
  distal_link: contact_3
c4:
  type: Cartesian
  distal_link: contact_4


    )";

    auto ik_pb = YAML::Load(ik_pb_txt);

    auto params = std::make_shared<XBot::Cartesian::Parameters>(1.0);

    auto ctx = std::make_shared<XBot::Cartesian::Context>(params, model);

    XBot::Cartesian::ProblemDescription pb(ik_pb, ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", pb, ctx);

    model->setJointPosition(model->getRobotState("home"));
    model->update();
    ci->reset(0);
    EXPECT_TRUE(ci->update(0, 0));

    auto constr = std::make_shared<CartesianConstraint>(ci);
    constr->bind(space);

    ProfilingData::instance().reset();
    TIC(sample_t);
    for(int i = 0; i < 100; i++)
    {
        auto q = constr->sample();

        EXPECT_LT(constr->value(q).norm(), 1e-3) <<
            q.transpose().format(3);
    }
    double sample_t = TOC(sample_t);
    ProfilingData::instance().print(std::cout, sample_t);
}



TEST_F(TestBasic, checkContactConstraint)
{
    model->getJoint(0)->setJointLimits(-Eigen::VectorXd::Ones(6), Eigen::VectorXd::Ones(6));

    auto space = std::make_shared<StateSpace>();

    space->addRobotConfigurationSpace(model);

    std::map<std::string, std::vector<int>> contacts = {
        {"contact_1", {0, 1, 2}},
        {"contact_2", {0, 1, 2}},
        {"contact_3", {0, 1, 2}},
        {"contact_4", {0, 1, 2}}
    };


    model->setJointPosition(model->getRobotState("home"));
    model->update();

    Eigen::VectorXd qstart = model->getJointPosition(), qgoal;

    auto constr = std::make_shared<ContactConstraint>(model, contacts);
    constr->bind(space);

    ASSERT_TRUE(constr->checkJacobian(model->getJointPosition()));

    ProfilingData::instance().reset();
    TIC(sample_t);
    for(int i = 0; i < 100; i++)
    {
        auto q = constr->sample();

        EXPECT_LT(constr->value(q).norm(), 1e-3) <<
            q.transpose().format(3);

        EXPECT_TRUE(space->checkBounds(q)) << q.transpose();

        ASSERT_TRUE(constr->checkJacobian(q));

        qgoal = q;

    }
    double sample_t = TOC(sample_t);
    ProfilingData::instance().print(std::cout, sample_t);

    Planner pl(space, {});
    EXPECT_TRUE(pl.solve(qstart, qgoal, 10.0, "RRTConnect"));
}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
