#include <gtest/gtest.h>

#include <matlogger2/matlogger2.h>

#include "planner/cartesio_ompl_planner.h"
#include "state_wrapper.h"

using namespace  XBot::Cartesian::Planning;
namespace ob = ompl::base;



class SphereConstraint : public ob::Constraint
{
public:
    SphereConstraint() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};


class TestBasics: public ::testing::Test {
    

protected:

     TestBasics(){
         
     }

     virtual ~TestBasics() {
     }

     virtual void SetUp() {
         
     }

     virtual void TearDown() {
     }
     
     
};


TEST_F(TestBasics, testStateWrapper)
{

    // Create the ambient space state space for the problem.
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);
      
    // Set bounds on the space.
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    rvss->setBounds(bounds);
    
    // Unconstrained state
    ob::ScopedState<> us(rvss);

    auto rvsw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::REALVECTOR, 3);
    
    Eigen::VectorXd set_value(3);
    set_value << 1.0, 2.0, 3.0;
    
    rvsw->setState(us.get(),set_value);
    
    Eigen::VectorXd get_value(3);
    rvsw->getState(us.get(),get_value);
    
    EXPECT_NEAR(set_value[0], get_value[0], 1e-6);
    EXPECT_NEAR(set_value[1], get_value[1], 1e-6);
    EXPECT_NEAR(set_value[2], get_value[2], 1e-6);
   
    
    // Create a shared pointer to sphere constraint.
    auto constraint = std::make_shared<SphereConstraint>();
    
    auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
    
    // Constrained state
    ob::ScopedState<> cs(css);
    
    auto csw = std::make_shared<StateWrapper>(StateWrapper::StateSpaceType::CONSTRAINED, 3);
    
    csw->setState(cs.get(),set_value);
    
    Eigen::VectorXd get_value_const(3);
    csw->getState(cs.get(),get_value_const);
    
    EXPECT_NEAR(set_value[0], get_value_const[0], 1e-6);
    EXPECT_NEAR(set_value[1], get_value_const[1], 1e-6);
    EXPECT_NEAR(set_value[2], get_value_const[2], 1e-6);
 
};


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
