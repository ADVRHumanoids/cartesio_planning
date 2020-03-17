#include <gtest/gtest.h>
#include <matlogger2/matlogger2.h>
#include <cartesian_interface/utils/LoadObject.hpp>

#include "ik/position_ik_solver.h"

using namespace XBot::Cartesian;

class TestProjection: public ::testing::Test {

protected:

    TestProjection() {
        // get model
        std::string config_path = TEST_CONFIG_PATH;
        XBot::ConfigOptions opt;
        opt.set_urdf_path ( config_path + "cogimon.urdf" );
        opt.set_srdf_path ( config_path + "cogimon.srdf" );
        opt.generate_jidmap();
        opt.set_parameter ( "is_model_floating_base", true );
        opt.set_parameter<std::string> ( "model_type", "RBDL" );
        _model = XBot::ModelInterface::getModel ( opt );

        // nonzero joint configuration
        const int nq = _model->getJointNum();
        Eigen::VectorXd q0;
        q0.setRandom ( nq );
        _model->setJointPosition ( q0 );
        _model->update();

        // custom manifold definition (hardcoded)
        auto l_sole_constr = XBot::Cartesian::MakeCartesian ( "l_sole" );
        auto r_sole_constr = XBot::Cartesian::MakeCartesian ( "r_sole" );
        ProblemDescription ik_prob ( {l_sole_constr, r_sole_constr} );

        // ci
        _ci = Utils::LoadObject<CartesianInterfaceImpl> ( "libCartesianOpenSot.so", "create_instance",
                _model, ik_prob );

        // solver
        _solver = std::make_shared<Planning::PositionCartesianSolver> ( _ci, ik_prob );

    }

    virtual ~TestProjection() {
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
    }

    XBot::ModelInterface::Ptr _model;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;
    XBot::Cartesian::Planning::PositionCartesianSolver::Ptr _solver;


};

//using namespace  XBot::Cartesian::Planning;

TEST_F ( TestProjection, testJacobian ) {
    const int nq = _model->getJointNum();

    Eigen::VectorXd q0;
    q0.setRandom ( nq );
    _model->setJointPosition ( q0 );
    _model->update();

    Eigen::VectorXd e0;
    _solver->getError ( e0 );

    Eigen::MatrixXd J;
    _solver->getJacobian ( J );

    Eigen::MatrixXd J_diff ( _solver->getSize(), nq );

    for ( int i = 0; i < nq; i++ ) {
        const double h = 1e-6;
        auto dq = Eigen::VectorXd::Unit ( nq, i ) *h;

        Eigen::VectorXd e;
        _model->setJointPosition ( q0 + dq );
        _model->update();
        _solver->getError ( e );

        J_diff.col ( i ) = ( e - e0 ) /h;

    }

    EXPECT_NEAR ( ( J-J_diff ).norm(), 0.0, 1e-4 );
};


int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
