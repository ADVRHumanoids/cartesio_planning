#include <gtest/gtest.h>
#include <matlogger2/matlogger2.h>

#include "constraints/cartesian_constraint.h"
#include "planner/cartesio_ompl_planner.h"

using namespace XBot;
using namespace XBot::Cartesian;

class TestManifold: public ::testing::Test {

protected:

    TestManifold()
    {
        // get model
        std::string config_path = TEST_CONFIG_PATH;
        XBot::ConfigOptions opt;
        opt.set_urdf_path(config_path + "teleop.urdf");
        opt.set_srdf_path(config_path + "teleop.srdf");
        opt.generate_jidmap();
        opt.set_parameter("is_model_floating_base", false);
        opt.set_parameter<std::string>("model_type", "RBDL");
        _model = XBot::ModelInterface::getModel(opt);

        // nonzero joint configuration
        const int nq = _model->getJointNum();
        Eigen::VectorXd q0;
        q0.setRandom(nq);
        _model->setJointPosition(q0);
        _model->update();

        // custom manifold definition (hardcoded)
        std::string ik_yaml =
                "stack: [[ArmZ]]             \n"
                "constraints: [jlims, vlims] \n"
                "ArmZ:                       \n"
                "  type: Cartesian           \n"
                "  distal_link: TCP          \n"
                "  indices: [2]              \n"
                "jlims:                      \n"
                "  type: JointLimits         \n"
                "vlims:                      \n"
                "  type: VelocityLimits      \n"
                "  limits: 0.1               \n";

        double ci_period = 1.0;
        auto ci_ctx = std::make_shared<Context>(
                    std::make_shared<Parameters>(ci_period),
                    _model);

        ProblemDescription ik_prob(YAML::Load(ik_yaml), ci_ctx);

        // ci
        _ci = CartesianInterfaceImpl::MakeInstance("OpenSot", ik_prob, ci_ctx);

        // constraint
        _constr = std::make_shared<Planning::CartesianConstraint>(
                    std::make_shared<Planning::PositionCartesianSolver>(_ci)
                    );

        Eigen::VectorXd qmin, qmax;
        _model->getJointLimits(qmin, qmax);
        _planner = std::make_shared<Planning::OmplPlanner>(
                    qmin, qmax, _constr, YAML::Node());

    }

    virtual ~TestManifold() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    ModelInterface::Ptr _model;
    CartesianInterfaceImpl::Ptr _ci;
    Planning::CartesianConstraint::Ptr _constr;
    Planning::OmplPlanner::Ptr _planner;


};

//using namespace  XBot::Cartesian::Planning;

TEST_F(TestManifold, testStartGoal)
{
    Eigen::VectorXd start(_model->getJointNum());
    start << 0, 1, 1, 0, 0;

    _model->setJointPosition(start);
    _model->update();
    _constr->reset();

    Eigen::VectorXd err;
    _constr->function(start, err);
    ASSERT_LE(err.norm(), 1e-4);

    Eigen::VectorXd goal(_model->getJointNum());
    goal << -1, 0, 0, 1, 1;

    ASSERT_TRUE(_constr->project(goal));

    _constr->function(goal, err);
    ASSERT_LE(err.norm(), 1e-4);

    std::string planner_type = "RRTstar";
    double timeout = 3.0;
    double threshold = 0.0;

    _planner->setStartAndGoalStates(start, goal, threshold);
    bool success = _planner->solve(timeout, planner_type);

    ASSERT_TRUE(success);

};


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
