#include <matlogger2/matlogger2.h>


 #include <ompl/control/SpaceInformation.h>
 #include <ompl/base/goals/GoalState.h>
 #include <ompl/base/spaces/SE2StateSpace.h>
 #include <ompl/control/spaces/RealVectorControlSpace.h>
 #include <ompl/control/planners/kpiece/KPIECE1.h>
 #include <ompl/control/planners/rrt/RRT.h>
 #include <ompl/control/planners/est/EST.h>
 #include <ompl/control/planners/pdst/PDST.h>
 #include <ompl/control/planners/syclop/GridDecomposition.h>
 #include <ompl/control/SimpleSetup.h>
 #include <ompl/config.h>
 #include <iostream>
 #include "state_wrapper.h"
 
 namespace ob = ompl::base;
 namespace oc = ompl::control;
 
 // a decomposition is only needed for SyclopRRT and SyclopEST
 class MyDecomposition : public oc::GridDecomposition
 {
 public:
     MyDecomposition(const int length, const ob::RealVectorBounds& bounds)
         : GridDecomposition(length, 2, bounds)
     {
     }
     void project(const ob::State* s, std::vector<double>& coord) const override
     {
         coord.resize(2);
         coord[0] = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
         coord[1] = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
     }
 
     void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
     {
         sampler->sampleUniform(s);
         s->as<ob::RealVectorStateSpace::StateType>()->values[0] = coord[0];
         s->as<ob::RealVectorStateSpace::StateType>()->values[1] = coord[1];
     }
 };
 
 bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
 {
 
     // extract the first component of the state and cast it to what we expect
     const auto pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
 
     // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
     return si->satisfiesBounds(state) && (!(pos[0] > 1 && pos[0] < 2 && pos[1] < 2 && pos[1] > -2));
 }
 
 void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
 {
     auto pos = start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
     auto ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
 
     auto r_result = result->as<ob::RealVectorStateSpace::StateType>()->values;
     r_result[0] = pos[0] + ctrl[0] * duration * cos(pos[2]);
     r_result[1] = pos[1] + ctrl[0] * duration * sin(pos[2]);
     r_result[2] = pos[2] + ctrl[1] * duration;
 }
 
 void plan()
 {
 
     // construct the state space we are planning in
     auto space(std::make_shared<ob::RealVectorStateSpace>(3));
 
     // set the bounds for the R^2 part of SE(2)
     ob::RealVectorBounds bounds(3);
     bounds.setLow(0, -0.5);
     bounds.setLow(1, -3.0);
     bounds.setLow(2, -3.14);
     bounds.setHigh(0, 5.5);
     bounds.setHigh(1, 3.0);
     bounds.setHigh(2, 3.14);
 
     space->setBounds(bounds);
 
     // create a control space
     auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
 
     // set the bounds for the control space
     ob::RealVectorBounds cbounds(2);
     cbounds.setLow(-1.0);
     cbounds.setHigh(1.0);
 
     cspace->setBounds(cbounds);
 
     // construct an instance of  space information from this control space
     auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

     // set state validity checking for this space
     si->setStateValidityChecker(
         [&si](const ob::State *state) { return isStateValid(si.get(), state); });
 
     // set the state propagation routine
     si->setStatePropagator(propagate);
 
     // create a start state
     ob::ScopedState<ob::RealVectorStateSpace> start(space);
     start->values[0] = 0.0;
     start->values[1] = 0.0;
     start->values[2] = 0.0;
 
     // create a goal state
     ob::ScopedState<ob::RealVectorStateSpace> goal(space);
     goal->values[0] = 5.0;
     goal->values[1] = 1.5;
     goal->values[2] = 0.0;
     
     // create a problem instance
     auto pdef(std::make_shared<ob::ProblemDefinition>(si));
 
     // set the start and goal states
     pdef->setStartAndGoalStates(start, goal, 0.1);
 
     // create a planner for the defined space
     auto planner(std::make_shared<oc::RRT>(si));
     //auto planner(std::make_shared<oc::EST>(si));
     //auto planner(std::make_shared<oc::KPIECE1>(si));
     //auto decomp(std::make_shared<MyDecomposition>(32, bounds));
     //auto planner(std::make_shared<oc::SyclopEST>(si, decomp));
     //auto planner(std::make_shared<oc::SyclopRRT>(si, decomp));
     
 
     // set the problem we are trying to solve for the planner
     planner->setProblemDefinition(pdef);
 
     // perform setup steps for the planner
     planner->setup();
 
 
     // print the settings for this space
     si->printSettings(std::cout);
 
     // print the problem settings
     pdef->print(std::cout);
 
     // attempt to solve the problem within ten seconds of planning time
     ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
 
     if (solved)
     {
         // get the goal representation from the problem definition (not the same as the goal state)
         // and inquire about the found path
         ob::PathPtr path = pdef->getSolutionPath();
         
        // Create a .mat file for post-analysis data
        // Create an instance to the logger variable
         auto logger = XBot::MatLogger2::MakeLogger("/tmp/ompl_control");
         logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

         // Transform the PathPtr in a PathGeometricPtr
         ompl::geometric::PathGeometric *path_geom = path->as<ompl::geometric::PathGeometric>();

         std::cout<<"path_geom->getStateCount(): "<<path_geom->getStateCount()<<std::endl;

         // Convert in Eigen::VectorXd
         XBot::Cartesian::Planning::StateWrapper sw(XBot::Cartesian::Planning::StateWrapper::StateSpaceType::REALVECTOR, 3);
         Eigen::VectorXd tmp(3);
         for(int j = 0; j < path_geom->getStateCount(); j++)
         {
             sw.getState(path_geom->getState(j), tmp);
             logger->add("computed_path", tmp);
         }

         // Add PlannerData elements
         ompl::control::PlannerData data(si);
         planner->getPlannerData(data);
         for (int i = 0; i < data.numVertices(); i++)
         {
             sw.getState(data.getVertex(i).getState(), tmp);
             logger->add("vertices", tmp);
         }
         
         std::cout << "Found solution:" << std::endl;
 
         // print the path to screen
         path->print(std::cout);

     }
     else
         std::cout << "No solution found" << std::endl;


 }
 
 int main()
 {
 
     plan();

     return 0;
 }
