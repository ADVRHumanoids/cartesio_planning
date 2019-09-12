#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>


class OmplPlanner
{

public:

    OmplPlanner();

private:

};


#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <matlogger2/matlogger2.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Manifold : public ob::Constraint
{

public:

    Manifold():
        Constraint(3, 1)
    {
    }

    virtual void function(const Eigen::Ref<const Eigen::VectorXd> &x,
                          Eigen::Ref<Eigen::VectorXd> out) const override
    {
        out[0] = x[0] - 0.9;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> & x,
                  Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        out << 1.0, 0.0, 0.0;
    }

//    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
//    {
//        x[0] = 0.9;
//    }

};


OmplPlanner::OmplPlanner()
{
    // construct the state space we are planning in
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);

    // set the bounds for the R^3 space
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto space_info = std::make_shared<ob::SpaceInformation>(space);

    // Create our sphere constraint.
    auto constraint = std::make_shared<Manifold>();

    // Combine the ambient space and the constraint into a constrained state space.
    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);

    // Define the constrained space information for this constrained state space.
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);

    // state validity check
    auto isStateValid = [](const ob::State * state)
    {
        // cast the abstract state type to the type we expect
        const Eigen::Map<Eigen::VectorXd>& r3state = *state->as<ob::ConstrainedStateSpace::StateType>();

        // check validity of state defined by pos & rot
        if(std::fabs(r3state[1]) < 0.3 && std::fabs(r3state[2] + 0.7) > 0.1)
        {
            return false;
        }

        // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
        return true;
    };

    csi->setStateValidityChecker(isStateValid);

    // create a random start state
    Eigen::VectorXd sv(3), gv(3);
    sv << 0.9, 0.9, 0.9;
    gv << 0.9, -0.9, 0.9;

    // Scoped states that we will add to simple setup.
    ob::ScopedState<> start(css);
    ob::ScopedState<> goal(css);

    // Copy the values from the vectors into the start and goal states.
    start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);


    // create a problem instance
    auto pdef = std::make_shared<ob::ProblemDefinition>(csi);

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // objective function
    pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(csi));

    // create a planner for the defined space
    auto planner = std::make_shared<og::RRTstar>(csi);

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();

    // print the settings for this space
    space_info->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->as<og::PathGeometric>()->interpolate(100);
        path->as<og::PathGeometric>()->printAsMatrix(std::cout);


        auto logger = XBot::MatLogger2::MakeLogger("/tmp/ompl_logger");

        for(int i = 0; i < path->as<og::PathGeometric>()->getStateCount(); i++)
        {
            auto * r3state = path->as<og::PathGeometric>()->getState(i)->as<ob::ConstrainedStateSpace::StateType>();
            logger->add("state", *r3state);
        }

    }
    else
        std::cout << "No solution found" << std::endl;
}
