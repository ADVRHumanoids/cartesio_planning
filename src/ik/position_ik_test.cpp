#include "position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>
#include <stability/stability_detection.h>
#include <constraints/validity_predicate_aggregate.h>
#include <functional>
#include <collisions/collision_detection.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "position_ik_test");
    ros::NodeHandle n("~");

    // obtain robot model from param server
    auto cfg = LoadOptions(LoadFrom::PARAM);
    auto model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome, qmin, qmax;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    model->getJointLimits(qmin, qmax);

    Eigen::Affine3d T;
    model->getPose("l_sole",T);
    model->setFloatingBasePose(T.inverse());
    model->update();


    // obtain ci object from param server
    auto ik_yaml = LoadProblemDescription(LoadFrom::PARAM);
    auto ik_prob = ProblemDescription(ik_yaml, model);

    std::string impl_name = "OpenSot";
    std::string path_to_shared_lib = XBot::Utils::FindLib("libCartesian" + impl_name + ".so", "LD_LIBRARY_PATH");
    if (path_to_shared_lib == "")
    {
        throw std::runtime_error("libCartesian" + impl_name + ".so must be listed inside LD_LIBRARY_PATH");
    }

    auto ci = SoLib::getFactoryWithArgs<CartesianInterfaceImpl>(path_to_shared_lib,
                                                                impl_name + "Impl",
                                                                model, ik_prob);

    if(!ci)
    {
        throw std::runtime_error("Unable to load solver '" + impl_name + "'");
    }

    // construct position-level ik solver
    XBot::Cartesian::Planning::PositionCartesianSolver::Ptr solver =
            std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci, ik_prob);

    // construct ros server class (mainly for markers and to publish TFs)
    RosServerClass ros_server(ci, model);

    // iteration callback (e.g. to step through the solution)
    solver->setIterCallback([&ros_server]()
    {
//        ros_server.run();
//        usleep(1e5);
    }
    );


    XBot::Cartesian::Planning::ConvexHullStability::PolygonFrames polyframes;
    polyframes.push_back("l_foot_lower_left_link");
    polyframes.push_back("l_foot_lower_right_link");
    polyframes.push_back("l_foot_upper_left_link");
    polyframes.push_back("l_foot_upper_right_link");
    polyframes.push_back("r_foot_lower_left_link");
    polyframes.push_back("r_foot_lower_right_link");
    polyframes.push_back("r_foot_upper_left_link");
    polyframes.push_back("r_foot_upper_right_link");
    XBot::Cartesian::Planning::ConvexHullStability ch(model, polyframes);


    XBot::Cartesian::Planning::CollisionDetection collision(model);


    // generate random goals respecting constraints
    int iter = 0;
    int failures = 0;
    auto t_start = ros::Time::now();
    auto t_print = t_start + ros::Duration(1.0);

    bool stable = false;

    XBot::Cartesian::Planning::ValidityPredicateAggregate valid;
    valid.add(std::bind(&XBot::Cartesian::Planning::ConvexHullStability::checkStability, ch), "convex_hull");
    valid.add(std::bind(&XBot::Cartesian::Planning::CollisionDetection::checkSelfCollisions, collision),
              "self_collisions", false);

    while(ros::ok())
    {
        iter++;

        Eigen::VectorXd qrand;
        if(!stable)
        {
            // generate random configuration
            qrand.setRandom(model->getJointNum()); // uniform in -1 < x < 1
            qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1

            qrand = qmin + qrand.cwiseProduct(qmax-qmin); // uniform in qmin < x < qmax
            qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)
        }
        else
            model->getJointPosition(qrand);


        // set it to the model
        model->setJointPosition(qrand);
        model->update();
        collision.update();

        // solve position-level ik
        if(!solver->solve())
        {
            failures++;
        }
        else
        {
            // this publishes TF under ci/ namespace


            stable = valid.checkAll();
        }

        ros_server.run();

        // every second we print statistics (generated states per second)
        auto now = ros::Time::now();
        double success_rate = 1.0-double(failures)/iter;
        if(now > t_print)
        {
            t_print = now + ros::Duration(1.0);
            printf("Producing %f goals per second \n"
                   "Success rate is %f \n",
                   (iter-failures)/(t_print-now).toSec(),
                   success_rate);
            iter = 0;
            failures = 0;
        }
//        if(success_rate <= 0.){
//            std::cout<<"RESETTING SOLVER"<<std::endl;
//            solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci, ik_prob);
//        }


    }

}
