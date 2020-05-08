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
                        std::shared_ptr<XBot::MatLogger2> logger,
                        const ompl::base::ConstraintPtr manifold = NULL):
        ompl::control::StatePropagator(si),
        _manifold(manifold),
        _sw(sw),
        _model(model),
        _logger(logger),
        _nh(nh) 
        {
            std::string problem_description_discrete_string;
            if(!_nh.getParam("problem_description_discrete_controls", problem_description_discrete_string))
            {
                ROS_ERROR("planner/problem_description_discrete_controls!");
                throw std::runtime_error("planner/problem_description_discrete_controls!");
            }

            auto ik_yaml_goal = YAML::Load(problem_description_discrete_string);

            double ci_period = 1.0;
            auto ci_ctx = std::make_shared<Context>(
                        std::make_shared<Parameters>(ci_period),
                        _model);

            auto ik_prob = ProblemDescription(ik_yaml_goal, ci_ctx);

            auto ci = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                        ik_prob, ci_ctx); 
            _solver = std::make_shared<PositionCartesianSolver>(ci);
        }
    
    void getEEPose(const ompl::base::State* start, Eigen::Affine3d& T) const
    {
        // Wrap the start state
        Eigen::VectorXd q;
        _sw->getState(start, q);
        
        // Update the model
        _model->setJointPosition(q);
        _model->update();
        
        // Extract end-effector position     
        _model->getPose("tool_exchanger", T);
        Eigen::Vector3d pos = T.translation();
        std::cout << pos[0] << " " << pos[1] << " " << pos[2] << "]" << std::endl;
    }    
    
    Eigen::VectorXd getJointPosition(Eigen::Affine3d T) const
    {
        
        _solver->setDesiredPose("tool_exchanger", T);
        
        if (_solver->solve())
        {
            std::cout << "Exact solution found" << std::endl;
            Eigen::VectorXd q; 
            _solver->getModel()->getJointPosition(q);
            return q;        
        }
        else
        {
            std::cout << "Approximate solution found" << std::endl;
            Eigen::VectorXd q;
            _solver->getModel()->getJointPosition(q);
            return q;
        }        
    }
        
    virtual void propagate (const ompl::base::State *start, const ompl::control::Control *control,
                   const double duration, ompl::base::State *result) const override
    {
        Eigen::Affine3d T;
        Eigen::Vector3d pose;
        int U = control->as<ompl::control::DiscreteControlSpace::ControlType>()->value;       
        _logger->add("control", U);
        
        std::cout << "Propagate starting from: [";
        getEEPose(start, T);
        
        // compute EE pose
        pose = T.translation();
        _logger->add("EE_pose", pose);        
        
        double gain = 0.01;
        
        std::cout << "Control action # " << U << std::endl;
        
//         if (sqrt(pose[0]*pose[0] + pose[1]*pose[1] + pose[2]*pose[2]) > 1.0)
//         {
//             switch (U)
//             {
//                 case (0):
//                     pose[0] -= gain;
//                 case (1):
//                     pose[1] -= gain;
//                 case (2):
//                     pose[2] -= gain;
//                 case (3):
//                     pose[0] -= gain;
//                 case (4):
//                     pose[1] -= gain;
//                 case (5):
//                     pose[2] -= gain;
//             }
//         }
//         else 
//         {
            switch (U)
            {
                case (0):
                    pose[0] -= gain;
                case (1):
                    pose[1] -= gain;
                case (2):
                    pose[2] -= gain;
                case (3):
                    pose[0] += gain;
                case (4):
                    pose[1] += gain;
                case (5):
                    pose[2] += gain;
            }
//         }
        
        std::cout << "Propagation ending to:   [" << pose[0] << " " << pose [1] << " " << pose[2] << "]" << std::endl;
        T.translation() = pose;
        Eigen::VectorXd q = getJointPosition(T);
        
        _sw->setState(result, q);
    };  
    
private:
    std::shared_ptr<StateWrapper> _sw;
    ompl::base::ConstraintPtr _manifold;
    XBot::ModelInterface::Ptr _model;
    std::shared_ptr<XBot::MatLogger2> _logger;
    ros::NodeHandle _nh;
    
    std::shared_ptr<PositionCartesianSolver> _solver;
    

};

} } } }
