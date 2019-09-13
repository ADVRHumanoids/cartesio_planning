#include "position_ik_solver.h"
#include <cartesian_interface/ros/RosServerClass.h>
#include <cartesian_interface/utils/LoadConfig.h>
#include <std_srvs/Trigger.h>

using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Utils;

bool g_trigger_solver = false;

bool trigger_solver_cb(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
    g_trigger_solver = true;
    return true;
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "position_ik_test");
    ros::NodeHandle n("~");

    auto srv = n.advertiseService("trigger_solver", trigger_solver_cb);

    auto cfg = LoadOptions(LoadFrom::PARAM);

    auto model = XBot::ModelInterface::getModel(cfg);

    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

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
    else
    {
        XBot::Logger::success("Loaded solver '%s'\n", impl_name.c_str());
    }


    XBot::Cartesian::Planning::PositionCartesianSolver solver(ci, {"l_sole", "r_sole"});
    RosServerClass ros_server(ci, model);
    solver.setIterCallback( [&ros_server](){ros_server.run();} );


    while(ros::ok())
    {
        ros::spinOnce();
        ros_server.run();

        if(true)
        {
            Eigen::VectorXd qrand;
            qrand.setRandom(model->getJointNum());
//            model->setJointPosition(qrand);
//            model->update();
            solver.solve();
            g_trigger_solver = false;
        }

        ros::Duration(0.1).sleep();
    }

}
