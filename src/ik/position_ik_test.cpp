#include "position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>

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
    XBot::Cartesian::Planning::PositionCartesianSolver solver(ci, {"TCP_L", "TCP_R", "l_sole", "r_sole"});

    // construct ros server class (mainly for markers and to publish TFs)
    RosServerClass ros_server(ci, model);

    // iteration callback (e.g. to step through the solution)
    solver.setIterCallback([&ros_server]()
    {
//        ros_server.run();
//        usleep(1e5);
    }
    );

    // generate random goals respecting constraints
    int iter = 0;
    int failures = 0;
    auto t_start = ros::Time::now();
    auto t_print = t_start + ros::Duration(1.0);
    while(ros::ok())
    {
        iter++;

        // generate random configuration
        Eigen::VectorXd qrand;
        qrand.setRandom(model->getJointNum()); // uniform in -1 < x < 1
        qrand = (qrand.array() + 1)/2.0; // uniform in 0 < x < 1

        qrand = qmin + qrand.cwiseProduct(qmax-qmin); // uniform in qmin < x < qmax
        qrand.head<6>().setRandom(); // we keep virtual joints between -1 and 1 (todo: improve)

        // set it to the model
        model->setJointPosition(qrand);
        model->update();

        // solve position-level ik
        if(!solver.solve())
        {
            failures++;
        }
        else
        {
            // this publishes TF under ci/ namespace
            ros_server.run();
        }

        // every second we print statistics (generated states per second)
        auto now = ros::Time::now();
        if(now > t_print)
        {
            t_print = now + ros::Duration(1.0);
            printf("Producing %f goals per second \n"
                   "Success rate is %f \n",
                   (iter-failures)/(t_print-now).toSec(),
                   1.0-double(failures)/iter);
            iter = 0;
            failures = 0;
        }

    }

}
