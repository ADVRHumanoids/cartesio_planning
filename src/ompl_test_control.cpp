#include <ompl/base/State.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>

#include <iostream>

#include <state_wrapper.h>
#include <matlogger2/matlogger2.h>

void propagate(const ompl::base::State *start, const ompl::control::Control *control, const double duration, ompl::base::State *result)
{
    const auto *se2state = start->as<ompl::base::SE2StateSpace::StateType>();
    const double x = se2state->getX();
    const double y = se2state->getY();
    const double rot = se2state->getYaw();
    const double *ctrl = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    
    result->as<ompl::base::SE2StateSpace::StateType>()->setXY(
        x + ctrl[0] * duration,
        y + ctrl[1] * duration);
    result->as<ompl::base::SE2StateSpace::StateType>()->setYaw(
        rot + ctrl[2] * duration);
}
 
bool isStateValid(std::shared_ptr<ompl::control::SpaceInformation> si, const ompl::base::State *state)
{
    auto *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    double x = se2state->getX();
    double y = se2state->getY();
    
    return (!(x > 1 && x < 2 && y > -2 && y < 2) && si->satisfiesBounds(state));
}

void plan()
{
    // Create an instance to the state space
    auto space = std::make_shared<ompl::base::SE2StateSpace>();
    
    // Define bounds only on x and y axis and assign to the state space
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, -0.5);
    bounds.setLow(1, -3.0);
    bounds.setHigh(0, 5.5);
    bounds.setHigh(1, 3.0);
    
    space->setBounds(bounds);
        
    // Define the control state space and relative bounds
    auto cspace = std::make_shared<ompl::control::RealVectorControlSpace>(space, 3);
    
    ompl::base::RealVectorBounds cbounds(3);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    
    cspace->setBounds(cbounds);
    
    // Create an intance to stape space information
    auto si = std::make_shared<ompl::control::SpaceInformation>(space, cspace);
    
    // Assign the validity checker
    si->setStateValidityChecker([&si] (const ompl::base::State *state) { return isStateValid(si, state); });
    
    // Assign propagator
    si->setStatePropagator(propagate);
    
//     si->setPropagationStepSize(0.3);  // Here you can modify the step_size (time)
        
    // Create an instance to the problem definition
    auto pdef = std::make_shared<ompl::base::ProblemDefinition>(si);

    // Set start and goal states
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
   
    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setX(5.0);
    goal->setY(1.5);
    
    pdef->setStartAndGoalStates(start, goal, 0.1);
    
    // Define the planner (RRT in this case)
    auto planner = std::make_shared<ompl::control::RRT>(si);
    
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
 
    // perform setup steps for the planner
    planner->setup();
  
    // Print the settings for this space
    si->printSettings(std::cout);
 
    // Print the problem settings
    pdef->print(std::cout);
    
    // Set an optimization objective 
    pdef->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));
 
    // Attempt to solve the problem within ten seconds of planning time
    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(30.0);
 
    if (solved)
    {
        // Get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = pdef->getSolutionPath();
         
        // Create a .mat file for post-analysis data
        // Create an instance to the logger variable
        auto logger = XBot::MatLogger2::MakeLogger("/home/luca/my_log/ompl_control");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
        
        // Transform the PathPtr in a PathGeometricPtr
        auto path_geom = path->as<ompl::geometric::PathGeometric>();
        
        // Convert in Eigen::VectorXd
        std::vector<Eigen::Vector3d> path_vect(path_geom->getStateCount());
        for(int j = 0; j < path_geom->getStateCount(); j++)
        {
            path_vect[j](0) = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getX();
            path_vect[j](1) = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getY();
            path_vect[j](2) = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
            
            logger->add("computed_path", path_vect[j]);
        }
                
        // Add PlannerData elements
        ompl::control::PlannerData data(si);
        planner->getPlannerData(data);
        std::vector<Eigen::Vector3d> vertices(data.numVertices());
        for (int i = 0; i < data.numVertices(); i++)
        {
            vertices[i](0) = data.getVertex(i).getState()->as<ompl::base::SE2StateSpace::StateType>()->getX();
            vertices[i](1) = data.getVertex(i).getState()->as<ompl::base::SE2StateSpace::StateType>()->getY();
            vertices[i](2) = data.getVertex(i).getState()->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
            
            logger->add("vertices", vertices[i]);
        }       
 
         // Print the path to screen
        std::cout << "Found solution:" << std::endl;
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