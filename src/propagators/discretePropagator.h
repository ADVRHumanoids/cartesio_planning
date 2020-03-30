#include "state_wrapper.h"
#include <ompl/base/State.h>
#include <ompl/control/Control.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ros/ros.h>
#include <XBotInterface/ModelInterface.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>

#include <cartesian_interface/CartesianInterface.h>
#include <cartesian_interface/Context.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "ik/position_ik_solver.h"

namespace XBot { namespace Cartesian { namespace Planning { namespace Propagators{

class discretePropagator : public ompl::control::StatePropagator{
public:
    discretePropagator (ompl::control::SpaceInformationPtr si,
                        std::shared_ptr<StateWrapper> sw,
                        XBot::ModelInterface::Ptr model, 
                        ros::NodeHandle& nh,
                        const ompl::base::ConstraintPtr manifold = NULL):
        ompl::control::StatePropagator(si),
        _manifold(manifold),
        _sw(sw),
        _model(model),
        _nh(nh) 
        {}
    
    void getEEPose(const ompl::base::State* start, Eigen::Affine3d& T) const
    {
        // Wrap the start state
        Eigen::VectorXd q;
        _sw->getState(start, q);
        
        // Update the model
        _model->setJointPosition(q);
        _model->update();
        
        // Extract end-effector position     
        _model->getPose("teleop_link5", T);
    }    
    
    Eigen::VectorXd getJointPosition(Eigen::Affine3d T) const
    {
        std::string problem_description_string;
        if(!_nh.getParam("problem_description_discrete_controls", problem_description_string))
        {
            ROS_ERROR("planner/problem_description_discrete_controls!");
            throw std::runtime_error("planner/problem_description_discrete_controls!");
        }

        auto ik_yaml_goal = YAML::Load(problem_description_string);

        double ci_period = 1.0;
        auto ci_ctx = std::make_shared<Context>(
                    std::make_shared<Parameters>(ci_period),
                    _model);

        auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);

        auto ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_prob, ci_ctx);
        
        PositionCartesianSolver solver(ci);
        solver.setDesiredPose("teleop_link5", T);
        solver.solve();  
        
        if (solver.solve())
        {
            Eigen::VectorXd q; 
            solver.getModel()->getJointPosition(q);
            return q;        
        }
        else 
            throw std::runtime_error("Unable to solve IK inside the discrete propagator");
        
    }
        
    virtual void propagate (const ompl::base::State *start, const ompl::control::Control *control,
                   const double duration, ompl::base::State *result) const override
    {
        Eigen::Affine3d T;
        Eigen::Vector3d pose;
        int U = control->as<ompl::control::DiscreteControlSpace::ControlType>()->value;        
        
        getEEPose(start, T);
        
        pose = T.translation();
        
        switch (U)
        {
            case (0):
                pose[0] = pose[0];
                pose[1] = pose[1];
                pose[2] = pose[2] + 0.05;
            case (1):
                pose[0] = pose[0];
                pose[1] = pose[1];
                pose[2] = pose[2] - 0.05;
            case (2):
                pose[0] = pose[0];
                pose[1] = pose[1] + 0.05;
                pose[2] = pose[2];
            case (3):
                pose[0] = pose[0];
                pose[1] = pose[1] - 0.05;
                pose[2] = pose[2];
            case (4):
                pose[0] = pose[0] + 0.05;
                pose[1] = pose[1];
                pose[2] = pose[2];
            case (5):
                pose[0] = pose[0] - 0.05;
                pose[1] = pose[1];
                pose[2] = pose[2];
        }
                
        T.translation() = pose;
        Eigen::VectorXd q = getJointPosition(T);
        
        _sw->setState(result, q);
    };  
    
private:
    std::shared_ptr<StateWrapper> _sw;
    ompl::base::ConstraintPtr _manifold;
    XBot::ModelInterface::Ptr _model;
    ros::NodeHandle _nh;

};

} } } }
