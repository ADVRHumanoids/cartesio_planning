#include <matlogger2/matlogger2.h>

#include <ompl/config.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>

#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>

#include <Eigen/Geometry>

#include <iostream>

#include <boost/math/constants/constants.hpp>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>

#include <ros/ros.h>

#include "state_wrapper.h"
#include "ik/position_ik_solver.h"
 
namespace ob = ompl::base;
namespace oc = ompl::control;
 
 
/////////////////////////////////////////////////////////////
// UNCOMMENT THIS FOR PLANNING IN CONTINUOUS CONTROL SPACE //
/////////////////////////////////////////////////////////////
// a decomposition is only needed for SyclopRRT and SyclopEST
//  class MyDecomposition : public oc::GridDecomposition
//  {
//  public:
//      MyDecomposition(const int length, const ob::RealVectorBounds& bounds)
//          : GridDecomposition(length, 2, bounds)
//      {
//      }
//      void project(const ob::State* s, std::vector<double>& coord) const override
//      {
//          coord.resize(2);
//          coord[0] = s->as<ob::RealVectorStateSpace::StateType>()->values[0];
//          coord[1] = s->as<ob::RealVectorStateSpace::StateType>()->values[1];
//      }
//  
//      void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const override
//      {
//          sampler->sampleUniform(s);
//          s->as<ob::RealVectorStateSpace::StateType>()->values[0] = coord[0];
//          s->as<ob::RealVectorStateSpace::StateType>()->values[1] = coord[1];
//      }
//  };
//  
//  bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
//  {
//  
//      // extract the first component of the state and cast it to what we expect
//      const auto pos = state->as<ob::RealVectorStateSpace::StateType>()->values;
//  
//      // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
//      return si->satisfiesBounds(state) && (!(pos[0] > 1 && pos[0] < 2 && pos[1] < 2 && pos[1] > -2));
//  }
//  
//  void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
//  {
//      auto pos = start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//      auto ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
//  
//      auto r_result = result->as<ob::RealVectorStateSpace::StateType>()->values;
//      r_result[0] = pos[0] + ctrl[0] * duration * cos(pos[2]);
//      r_result[1] = pos[1] + ctrl[0] * duration * sin(pos[2]);
//      r_result[2] = pos[2] + ctrl[1] * duration;
//  }
//  
//  void plan()
//  {
//  
//      // construct the state space we are planning in
//      auto space(std::make_shared<ob::RealVectorStateSpace>(3));
//  
//      // set the bounds for the R^2 part of SE(2)
//      ob::RealVectorBounds bounds(3);
//      bounds.setLow(0, -0.5);
//      bounds.setLow(1, -3.0);
//      bounds.setLow(2, -3.14);
//      bounds.setHigh(0, 5.5);
//      bounds.setHigh(1, 3.0);
//      bounds.setHigh(2, 3.14);
//  
//      space->setBounds(bounds);
//  
//      // create a control space
//      auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
//  
//      // set the bounds for the control space
//      ob::RealVectorBounds cbounds(2);
//      cbounds.setLow(-1.0);
//      cbounds.setHigh(1.0);
//  
//      cspace->setBounds(cbounds);
//  
//      // construct an instance of  space information from this control space
//      auto si(std::make_shared<oc::SpaceInformation>(space, cspace));
// 
//      // set state validity checking for this space
//      si->setStateValidityChecker(
//          [&si](const ob::State *state) { return isStateValid(si.get(), state); });
//  
//      // set the state propagation routine
//      si->setStatePropagator(propagate);
//  
//      // create a start state
//      ob::ScopedState<ob::RealVectorStateSpace> start(space);
//      start->values[0] = 0.0;
//      start->values[1] = 0.0;
//      start->values[2] = 0.0;
//  
//      // create a goal state
//      ob::ScopedState<ob::RealVectorStateSpace> goal(space);
//      goal->values[0] = 5.0;
//      goal->values[1] = 1.5;
//      goal->values[2] = 0.0;
//      
//      // create a problem instance
//      auto pdef(std::make_shared<ob::ProblemDefinition>(si));
//  
//      // set the start and goal states
//      pdef->setStartAndGoalStates(start, goal, 0.1);
//  
//      // create a planner for the defined space
//      auto planner(std::make_shared<oc::RRT>(si));
//      //auto planner(std::make_shared<oc::EST>(si));
//      //auto planner(std::make_shared<oc::KPIECE1>(si));
//      //auto decomp(std::make_shared<MyDecomposition>(32, bounds));
//      //auto planner(std::make_shared<oc::SyclopEST>(si, decomp));
//      //auto planner(std::make_shared<oc::SyclopRRT>(si, decomp));
//      
//  
//      // set the problem we are trying to solve for the planner
//      planner->setProblemDefinition(pdef);
//  
//      // perform setup steps for the planner
//      planner->setup();
//  
//  
//      // print the settings for this space
//      si->printSettings(std::cout);
//  
//      // print the problem settings
//      pdef->print(std::cout);
//  
//      // attempt to solve the problem within ten seconds of planning time
//      ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
//  
//      if (solved)
//      {
//          // get the goal representation from the problem definition (not the same as the goal state)
//          // and inquire about the found path
//          ob::PathPtr path = pdef->getSolutionPath();
//          
//         // Create a .mat file for post-analysis data
//         // Create an instance to the logger variable
//          auto logger = XBot::MatLogger2::MakeLogger("/tmp/ompl_control");
//          logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
// 
//          // Transform the PathPtr in a PathGeometricPtr
//          ompl::geometric::PathGeometric *path_geom = path->as<ompl::geometric::PathGeometric>();
// 
//          std::cout<<"path_geom->getStateCount(): "<<path_geom->getStateCount()<<std::endl;
// 
//          // Convert in Eigen::VectorXd
//          XBot::Cartesian::Planning::StateWrapper sw(XBot::Cartesian::Planning::StateWrapper::StateSpaceType::REALVECTOR, 3);
//          Eigen::VectorXd tmp(3);
//          for(int j = 0; j < path_geom->getStateCount(); j++)
//          {
//              sw.getState(path_geom->getState(j), tmp);
//              logger->add("computed_path", tmp);
//          }
// 
//          // Add PlannerData elements
//          ompl::control::PlannerData data(si);
//          planner->getPlannerData(data);
//          for (int i = 0; i < data.numVertices(); i++)
//          {
//              sw.getState(data.getVertex(i).getState(), tmp);
//              logger->add("vertices", tmp);
//          }
//          
//          std::cout << "Found solution:" << std::endl;
//  
//          // print the path to screen
//          path->print(std::cout);
// 
//      }
//      else
//          std::cout << "No solution found" << std::endl;
// 
// 
//  }
//  
//  int main()
//  {
//  
//      plan();
// 
//      return 0;
//  }
 
///////////////////////////////////////////////////////////
// UNCOMMENT THIS FOR PLANNING IN DISCRETE CONTROL SPACE //
///////////////////////////////////////////////////////////
/*
void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    auto se2state = start->as<ompl::base::SE2StateSpace::StateType>();
    auto pos = se2state->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    auto rot = se2state->as<ompl::base::SO2StateSpace::StateType>(1)->value;
    auto ctrl = control->as<oc::DiscreteControlSpace::ControlType>()->value;

    auto r_result = result->as<ob::SE2StateSpace::StateType>();
    switch (ctrl)
    {
        case 0:
            r_result->setXY(pos[0] + 0.01, pos[1]);
            r_result->setYaw(rot);
            break;
        case 1:
            r_result->setXY(pos[0] - 0.01, pos[1]);
            r_result->setYaw(rot);
            break;
        case 2:
            r_result->setXY(pos[0], pos[1] + 0.01);
            r_result->setYaw(rot);
            break;
        case 3:
            r_result->setXY(pos[0], pos[1] - 0.01);
            r_result->setYaw(rot);
            break;
    }        
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{

    // extract the first component of the state and cast it to what we expect
    auto se2state = state->as<ob::SE2StateSpace::StateType>();
    const auto pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;

    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (!(pos[0] > 1 && pos[0] < 2 && pos[1] < 2 && pos[1] > -2));
}

void plan()
{
    auto space = std::make_shared<ompl::base::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -0.5);
    bounds.setLow(1, -3.0);
    bounds.setHigh(0, 5.5);
    bounds.setHigh(1, 3.0);
    space->setBounds(bounds);
    
    auto cspace = std::make_shared<ompl::control::DiscreteControlSpace>(space, 0, 3);
    
    // construct an instance of  space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // set state validity checking for this space
    si->setStateValidityChecker(
        [&si](const ob::State *state) { return isStateValid(si.get(), state); });
 
    // set the state propagation routine
    si->setStatePropagator(propagate);
 
    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);
 
    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(5.0);
    goal->setY(1.5);
    goal->setYaw(0.0);
     
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
        auto logger = XBot::MatLogger2::MakeLogger("/home/luca/my_log/ompl_control");
        logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);

        // Transform the PathPtr in a PathGeometricPtr
        ompl::geometric::PathGeometric *path_geom = path->as<ompl::geometric::PathGeometric>();

        std::cout<<"path_geom->getStateCount(): "<<path_geom->getStateCount()<<std::endl;

        // Convert in Eigen::VectorXd
        XBot::Cartesian::Planning::StateWrapper sw(XBot::Cartesian::Planning::StateWrapper::StateSpaceType::SE2SPACE, 3);
        Eigen::VectorXd tmp(3);
        for(int j = 0; j < path_geom->getStateCount(); j++)
        {
//             sw.getState(path_geom->getState(j), tmp);
            tmp[0] = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getX();
            tmp[1] = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getY();
            tmp[2] = path_geom->getState(j)->as<ompl::base::SE2StateSpace::StateType>()->getYaw();
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


}*/

///////////////////////////////////////////////////////////
// UNCOMMENT THIS FOR PLANNING WITH 2 SE2 STATE SPACES   //
///////////////////////////////////////////////////////////

class stepSampler : public ompl::control::ControlSampler {
public:
    stepSampler (const ompl::control::ControlSpace* cspace):
    ompl::control::ControlSampler(cspace) {}   
    
    void sample(ompl::control::Control* control) override
    {
        control->as<oc::DiscreteControlSpace::ControlType>()->value = rng_.uniformInt(
                space_->as<oc::DiscreteControlSpace>()->getLowerBound(), space_->as<oc::DiscreteControlSpace>()->getUpperBound());
    }
    
    void sample(ompl::control::Control* control, const ompl::base::State* state) override
    {
        sample(control);
    }
    
    void sampleNext(ompl::control::Control *control, const ompl::control::Control *previous) override
    {
        do 
        {
            sample(control);
        }
        while (control->as<oc::DiscreteControlSpace::ControlType>()->value == previous->as<oc::DiscreteControlSpace::ControlType>()->value);
    } 
    
    void sampleNext(ompl::control::Control* control, const ompl::control::Control* previous, const ompl::base::State* state) override
    {
        sampleNext(control,previous);
    }
};    

ompl::control::ControlSamplerPtr getSampler(const ompl::control::ControlSpace* cspace)
{
    return std::make_shared<stepSampler>(cspace);
}

double findRelativeAngle(const double angle1, const double angle2)
{
    double res1 = abs(angle1 - angle2);
    double res2 = abs(boost::math::constants::pi<double>()*2 - (angle1 - angle2));
    
    return std::min<double>(res1, res2);
}

bool checkForFeetCrossing(const double* l_pos, const double* r_pos, const double l_rot = 0)
{
    const double xRel_w = r_pos[0] - l_pos[0];
    const double yRel_w = r_pos[1] - l_pos[1];
    const double yRel_lf = -xRel_w * sin(l_rot) + yRel_w * cos(l_rot);
    
    return (yRel_lf < -0.15);
}


bool isStateValid(const oc::SpaceInformation* si, const ompl::base::State* state)
{
    auto s = state->as<ompl::base::CompoundStateSpace::StateType>();
//     auto s_f1 = s->as<ompl::base::SE2StateSpace::StateType>(0);  // right foot       Plan with foot orientation
//     auto s_f2 = s->as<ompl::base::SE2StateSpace::StateType>(1);  // left foot
    auto s_f1 = s->as<ompl::base::RealVectorStateSpace::StateType>(0);  // right foot       Plan only on foot position keeping 0-orientation
    auto s_f2 = s->as<ompl::base::RealVectorStateSpace::StateType>(1);  // left_foot
//     const double *pos1 = s_f1->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    const double *pos1 = s_f1->values;
//     const double rot1 = s_f1->as<ompl::base::SO2StateSpace::StateType>(1)->value;  // Needed when planning with orientation
//     const double *pos2 = s_f2->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
    const double *pos2 = s_f2->values;
//     const double rot2 = s_f2->as<ompl::base::SO2StateSpace::StateType>(1)->value;  // Needed when planning with orientation
    
    double distance_x = sqrt((pos1[0] - pos2[0]) * (pos1[0] - pos2[0]));
    double distance_y = sqrt((pos1[1] - pos2[1]) * (pos1[1] - pos2[1]));
//     double orientation = findRelativeAngle(rot1, rot2);
    
    return (si->satisfiesBounds(state) &&
            distance_x < 0.5 &&
            distance_y < 0.35/*&& 
            orientation < boost::math::constants::pi<double>()/6*/ && 
            !(pos1[0] > 1 && pos1[0] < 2 && pos1[1] < 2 && pos1[1] > -2) && 
            !(pos2[0] > 1 && pos2[0] < 2 && pos2[1] < 2 && pos2[1] > -2) &&
            checkForFeetCrossing(pos2, pos1));
}

void propagate(const oc::SpaceInformation* si, const ob::State* start, const oc::Control* control, const double duration, ob::State* result)
{
    // Cast the control to the desired types
    auto ctrl = control->as<ompl::control::CompoundControlSpace::ControlType>();
    int foot_sel = ctrl->as<oc::DiscreteControlSpace::ControlType>(0)->value;
    double* step = ctrl->as<oc::RealVectorControlSpace::ControlType>(1)->values;
    
    // Cast the states
    auto r = result->as<ob::CompoundStateSpace::StateType>();
//     auto r_f1 = r->as<ob::SE2StateSpace::StateType>(0);  // Plan with foot orientation
//     auto r_f2 = r->as<ob::SE2StateSpace::StateType>(1);
    auto r_f1 = r->as<ob::RealVectorStateSpace::StateType>(0)->values;  // Plan only on foot position keeping 0-orientation
    auto r_f2 = r->as<ob::RealVectorStateSpace::StateType>(1)->values;
    si->getStateSpace()->copyState(result, start);
    
    // Propagate
    if (foot_sel == 1)
    {
//         r_f1->setXY(r_f1->getX() + duration * step[0], r_f1->getY() + duration * step[1]);  // Plan with foot orientation
//         r_f1->setYaw(r_f1->getYaw() + duration * step[2]);
        r_f1[0] += duration * step[0];  // Plan only on foot position keeping 0-orientation
        r_f1[1] += duration * step[1];
    }
    else
    {
//         r_f2->setXY(r_f2->getX() + duration * step[0], r_f2->getY() + duration * step[1]);  // Plan with foot orientation
//         r_f2->setYaw(r_f2->getYaw() + duration * step[2]);
        r_f2[0] += duration * step[0];  // Plan only on foot position keeping 0-orientation
        r_f2[1] += duration * step[1];
    }
}

void loadModel(std::shared_ptr<XBot::ModelInterface>& model)
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    model = XBot::ModelInterface::getModel(cfg);
}

void solveIK(ompl::geometric::PathGeometric* solution, 
             std::shared_ptr<XBot::ModelInterface> model,
             std::vector<Eigen::VectorXd>& q, 
             ros::NodeHandle& nh,
             std::shared_ptr<XBot::Cartesian::Planning::PositionCartesianSolver> solver)
{

    
    auto sw = std::make_shared<XBot::Cartesian::Planning::StateWrapper>(XBot::Cartesian::Planning::StateWrapper::StateSpaceType::REALVECTOR, 2);
    Eigen::VectorXd rfoot, lfoot;
    Eigen::Affine3d T_r, T_l, T_com;
    
    T_r.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T_l.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T_com.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    
    for (int i = 0; i < solution->getStateCount(); i++)
    {
        auto state = solution->getState(i)->as<ompl::base::CompoundStateSpace::StateType>();
        auto r_foot = state->as<ompl::base::RealVectorStateSpace::StateType>(0);
        auto l_foot = state->as<ompl::base::RealVectorStateSpace::StateType>(1);
        sw->getState(r_foot, rfoot);
        sw->getState(l_foot, lfoot);
                
        T_r.translation() << rfoot[0], rfoot[1], 0;
        T_l.translation() << lfoot[0], lfoot[1], 0;
        T_com.translation() << (rfoot[0] + lfoot[0])/2, (rfoot[1] + lfoot[1])/2, 0;
        
        solver->setDesiredPose("l_sole", T_l);
        solver->setDesiredPose("r_sole", T_r);
        solver->setDesiredPose("Com", T_com);
        
        if (solver->solve())
        {
            solver->getModel()->getJointPosition(q[i]);
        }
        else
        {
            std::cout << "Approximate solution found for state " << i << std::endl;
            solver->getModel()->getJointPosition(q[i]);
        }
        
    }
}

void solveIK1(ompl::base::State* state, 
              ompl::control::Control* control,
              std::vector<Eigen::VectorXd>& q,
              std::shared_ptr<XBot::Cartesian::Planning::PositionCartesianSolver> solver,
              int n_step)
{
    Eigen::Vector3d com_start, com_goal, com_traj;
    
    // Cast state and control to the desired type
    auto s = state->as<ob::CompoundStateSpace::StateType>();
    auto rfoot = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto lfoot = s->as<ob::RealVectorStateSpace::StateType>(1)->values;
    
    auto c = control->as<oc::CompoundControlSpace::ControlType>();
    auto sel = c->as<oc::DiscreteControlSpace::ControlType>(0)->value;
    
    // Set start and goal position of the com depending on the swing foot
    com_start << (rfoot[0] + lfoot[0])/2, (rfoot[1] + lfoot[1])/2, 0;
    if (sel == 1)
        com_goal << lfoot[0], lfoot[1], 0;
    else
        com_goal << rfoot[0], rfoot[1], 0;
    
    // Compute the trajectory
    Eigen::Vector3d step = (com_goal - com_start)/n_step; 
    Eigen::Affine3d T_com;
    T_com.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::VectorXd joint_pos;

    for (int j = 0; j <= n_step; j ++)
    {
        com_traj = com_start + j*step;
        T_com.translation() = com_traj;
        
        solver->setDesiredPose("Com", T_com);
        if (solver->solve())
        {
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
        else
        {
            std::cout << "Warning: approximate solution found while creating first trajectory" << std::endl;
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
    }
    
}

void solveIK2(ompl::geometric::PathGeometric path,
              ompl::control::Control* control,
              std::vector<Eigen::VectorXd>& q,
              std::shared_ptr<XBot::Cartesian::Planning::PositionCartesianSolver> solver)
{
    auto sel = control->as<oc::CompoundControlSpace::ControlType>()->as<oc::DiscreteControlSpace::ControlType>(0)->value;
    
    double clearence = 0.1;
    
    Eigen::Affine3d T_sf;
    T_sf.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    double z = 0;
    
    for (int i = 0; i < path.getStateCount(); i ++)
    {
        auto swing_foot = path.getState(i)->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::RealVectorStateSpace::StateType>(sel - 1)->values;
        z = -(0.4*clearence*(0.1*i)*(0.1*i)) + 0.4*clearence*(0.1*i);
                
        T_sf.translation() << swing_foot[0], swing_foot[1], z;
        
        if (sel == 1)
            solver->setDesiredPose("r_sole", T_sf);
        else
            solver->setDesiredPose("l_sole", T_sf);
        
        Eigen::VectorXd joint_pos;
        if (solver->solve())
        {
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
        else
        {
            std::cout << "Warning: approximate solution found while creating the second trajectory" << std::endl;
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
    }  
}

void solveIK3(ompl::base::State* state, 
              ompl::control::Control* control,
              std::vector<Eigen::VectorXd>& q,
              std::shared_ptr<XBot::Cartesian::Planning::PositionCartesianSolver> solver,
              int n_step)
{
    // Cast state and control to the desired type
    auto s = state->as<ob::CompoundStateSpace::StateType>();
    auto rfoot = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
    auto lfoot = s->as<ob::RealVectorStateSpace::StateType>(1)->values;
    
    Eigen::Vector3d com_start, com_goal, com_traj;
    auto sel = control->as<ompl::control::CompoundControlSpace::ControlType>()->as<ompl::control::DiscreteControlSpace::ControlType>(0)->value;
    
    if (sel == 1)
        com_start << lfoot[0], lfoot[1], 0;
    else
        com_start << rfoot[0], rfoot[1], 0;
    
    com_goal << (lfoot[0] + rfoot[0])/2, (lfoot[1] + rfoot[1])/2, 0;
    
    // Compute the trajectory
    Eigen::Vector3d step = (com_goal - com_start)/n_step; 
    Eigen::Affine3d T_com;
    T_com.linear() << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::VectorXd joint_pos;

    for (int j = 0; j <= n_step; j ++)
    {
        com_traj = com_start + j*step;
        T_com.translation() = com_traj;
        
        solver->setDesiredPose("Com", T_com);
        if (solver->solve())
        {
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
        else
        {
            std::cout << "Warning: approximate solution found while creating first trajectory" << std::endl;
            solver->getModel()->getJointPosition(joint_pos);
            q.push_back(joint_pos);
        }
    }
    
    
}

std::shared_ptr<ompl::base::Path> plan()
{
    // Define state space for first foot
//     auto foot1 = std::make_shared<ompl::base::SE2StateSpace>();         // Plan with foot orientation
    auto foot1 = std::make_shared<ompl::base::RealVectorStateSpace>(2);  // Plan only on foot position keeping 0-orientation
    ob::RealVectorBounds bounds_foot1(2);
    bounds_foot1.setLow(-3.0);
    bounds_foot1.setHigh(3.0);
    foot1->setBounds(bounds_foot1);
    
    // Define state space for second foot
//     auto foot2 = std::make_shared<ompl::base::SE2StateSpace>();          // Plan with foot orientation
    auto foot2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);  // Plan only on foot position keeping 0-orientation
    ob::RealVectorBounds bounds_foot2(2);
    bounds_foot2.setLow(-3.0);
    bounds_foot2.setHigh(3.0);
    foot2->setBounds(bounds_foot2);
    
    // Combine the two state spaces
    ob::StateSpacePtr space = foot1 + foot2;
    
    // Create compound control space
    auto foot_sel = std::make_shared<ompl::control::DiscreteControlSpace>(space, 1, 2);
    foot_sel->setControlSamplerAllocator(getSampler);
    
//     auto step = std::make_shared<ompl::control::RealVectorControlSpace>(space, 3);  // Plan with 
//     ob::RealVectorBounds step_bounds(3);                                            // foot orientation
    auto step = std::make_shared<ompl::control::RealVectorControlSpace>(space, 2);  // Plan only on foot position   
    ob::RealVectorBounds step_bounds(2);                                            // keeping 0-orientation
    step_bounds.setLow(-1);
    step_bounds.setHigh(1);
    step->setBounds(step_bounds);
    
    auto cspace = std::make_shared<ompl::control::CompoundControlSpace>(space);
    cspace->addSubspace(foot_sel);
    cspace->addSubspace(step);    
    
    auto si = std::make_shared<oc::SpaceInformation>(space, cspace);
    si->setStateValidityChecker([&si](const ob::State* state){ return(isStateValid(si.get(), state)); });
    si->setStatePropagator([&si](const ob::State* state, const oc::Control* control, const double duration, ob::State* result){ return propagate(si.get(), state, control, duration, result); });
    
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    
    ob::ScopedState<> start(space);
//     start[0] = start[3] = 0.0;
//     start[1] = -0.1;
//     start[4] = -start[1];
//     start[2] = start[5] = 0;
    start[0] = start[2] = 0.0;
    start[1] = -0.1;
    start[3] = -start[1];
    
    ob::ScopedState<> goal(space);
//     goal[0] = goal[3] = 2.5;
//     goal[1] = start[1];
//     goal[4] = start[4];
//     goal[2] = goal[5] = 0;
    goal[0] = goal[2] = 2.5;
    goal[1] = start[1];
    goal[3] = start[3];
    
    pdef->setStartAndGoalStates(start, goal);
    
    auto planner = std::make_shared<ompl::control::RRT>(si); // Plan with controls
//     auto planner = std::make_shared<ompl::geometri::RRTConnect>(si); // Plan with states
    planner->setProblemDefinition(pdef);
    
    planner->setup();
 
    si->printSettings(std::cout);
    pdef->print(std::cout);
    ob::PlannerStatus solved = planner->ob::Planner::solve(15.0);
    
    if (solved)
    {
        std::cout << "Solution Found" << std::endl;
        
        auto path = pdef->getSolutionPath();
        auto geom_path = path->as<ompl::geometric::PathGeometric>();
        auto control_path = path->as<oc::PathControl>();
        
//         XBot::Cartesian::Planning::StateWrapper sw(XBot::Cartesian::Planning::StateWrapper::StateSpaceType::SE2SPACE, 3);
        
//         auto logger = XBot::MatLogger2::MakeLogger("/home/luca/my_log/2feet");
//         logger->set_buffer_mode(XBot::VariableBuffer::Mode::circular_buffer);
//         Eigen::VectorXd r_foot(si->getStateDimension()/2);
//         Eigen::VectorXd l_foot(si->getStateDimension()/2);
//         
//         for (int i = 0; i < geom_path->getStateCount(); i++)
//         {
//             auto state = geom_path->getState(i)->as<ob::CompoundStateSpace::StateType>();
// //             auto rf = state->as<ob::SE2StateSpace::StateType>(0);
// //             auto lf = state->as<ob::SE2StateSpace::StateType>(1);
//             auto rf = state->as<ob::RealVectorStateSpace::StateType>(0)->values;  // Plan only on foot position keeping 0-orientation
//             auto lf = state->as<ob::RealVectorStateSpace::StateType>(1)->values;
//             
//             r_foot[0] = rf[0];
//             r_foot[1] = rf[1];
//             l_foot[0] = lf[0];
//             l_foot[1] = lf[1];
//             
// //             sw.getState(rf, r_foot);
// //             sw.getState(lf, l_foot);
//             logger->add("r_foot", r_foot);
//             logger->add("l_foot", l_foot);
//         }
//         
//         for (int i = 0; i < control_path->getControlCount(); i++)
//         {
//             auto ctrl = control_path->getControl(i)->as<oc::CompoundControlSpace::ControlType>();
//             int foot_sel = ctrl->as<oc::DiscreteControlSpace::ControlType>(0)->value;
//             
//             logger->add("foot_sel", foot_sel);
//         }
        
        path->print(std::cout);
        return path;
    }
    else
    {
        std::cout << "Solution Not Found!" << std::endl;   
    }
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ompl_control_test");
    ros::NodeHandle nh;
    
    // Load the robot model
    std::shared_ptr<XBot::ModelInterface> model;
    loadModel(model);
    
    // Load the home state and publish
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    
    std::cout << "Home Position: \n" << qhome << std::endl;
    
    XBot::Cartesian::Utils::RobotStatePublisher RobotStatePublisher(model);
    RobotStatePublisher.publishTransforms(ros::Time::now(), "");

    // Compute the plan for the feet
    std::shared_ptr<ompl::base::Path> solution = plan();
    auto solution_geom = solution->as<ompl::geometric::PathGeometric>();
    auto solution_ctrl = solution->as<ompl::control::PathControl>();
  
    
    // Create an instance of the CartesianInterface
    std::string problem_description;
    if(!nh.getParam("problem_description", problem_description))
    {
        ROS_ERROR("planner/problem_description!");
        throw std::runtime_error("planner/problem_description!");
    }

    auto ik_yaml_goal = YAML::Load(problem_description);

    double ci_period = 1.0;
    auto ci_ctx = std::make_shared<XBot::Cartesian::Context>(
                  std::make_shared<XBot::Cartesian::Parameters>(ci_period),
                  model);
    auto ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx); 
    auto solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci);
    
    
    // THESE 2 ROWS CREATE PROJECTS THE ROBOT ONLY ON THE FEET POSITION FOUND WITH THE PLANNER
//     std::vector<Eigen::VectorXd> q(solution_geom->getStateCount());

//     solveIK(solution_geom, model, q, nh, solver);
    
    
    // THIS SECTION CREATES A STATIC WALK GOING THROUGH THE FOOT STEP FOUND WITH THE PLANNER
    int n_step = 10;
    std::vector<Eigen::VectorXd> q_vect;
    
    for (int i = 0; i < solution_geom->getStateCount()-1; i++)
    {
        // Solution split in 3 parts:
        // 1) COM moves in corrispondence of the support foot
        // 2) Swing foot moves to the computed position
        // 3) COM move in the middle of the two feet
        
        //1)
        solveIK1(solution_geom->getState(i), solution_ctrl->getControl(i), q_vect, solver, n_step);
        //2)
        // Construct a geometric path segment between the state i and the state i+1
        ompl::geometric::PathGeometric sw_traj(solution->getSpaceInformation(), 
                                               solution_geom->getState(i),
                                               solution_geom->getState(i+1));
        // Interpolate
        sw_traj.interpolate(n_step+1);
        solveIK2(sw_traj, solution_ctrl->getControl(i), q_vect , solver);
        
        //3)
        solveIK3(solution_geom->getState(i+1), solution_ctrl->getControl(i), q_vect, solver, n_step);
    }
    
    
    int i = 0;
    ros::Rate rate(10);
    while (ros::ok())
    {
//         std::cout << "Start ros loop number " << i << std::endl;
        model->setJointPosition(q_vect[i]);
        model->update();
        
        RobotStatePublisher.publishTransforms(ros::Time::now(), "");
//         std::cout << "Transform " << i << " sent!" << std::endl;
        i++;
        
        if (i == q_vect.size())
            i = 0;
        
        rate.sleep();
    }
    
    
    return 0;
}    
