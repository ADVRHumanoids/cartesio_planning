#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include "validity_checker/stability/centroidal_statics.h"
#include <std_srvs/Empty.h>
#include <cartesio_planning/SetContactFrames.h>

using namespace XBot::Cartesian::Planning;





int main(int argc, char ** argv)
{
    ros::init(argc, argv, "centroidal_statics_test");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("r_sole");
    double mu = 1.;
    Eigen::Vector2d xlims, ylims;
    xlims[0] = -0.05; xlims[1] = 0.1;
    ylims[0] = -0.05; ylims[1] = 0.05;
    CentroidalStatics::Ptr cs = std::make_shared<CentroidalStatics>(model, links_in_contact, mu, true, xlims, ylims);
    CentroidalStaticsROS csROS(model, cs, nhp);

    auto on_js_received = [&csROS, &cs, model](const sensor_msgs::JointStateConstPtr& msg)
    {
        Eigen::VectorXd q(model->getJointNum()); q.setZero();
        for(int i = 0; i < msg->name.size(); i++)
            q[i] = msg->position[i];

        model->setJointPosition(q);
        model->update();

//        if(cs.checkStability(1e-3))
//            ROS_INFO("STABLE!");
//        else
//            ROS_WARN("NOT STABLE!");


        std::map<std::string, Eigen::Vector6d> links_in_contact = cs->getForces();

        for(auto contact : links_in_contact)
        {
            Eigen::Affine3d T;
            model->getPose(contact.first, T);
            if(!cs->setContactRotationMatrix(contact.first, T.linear()))
                ROS_ERROR("Can not set rotation for link %s", contact.first);
        }

        csROS.publish();
    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("cartesian/solution", 1, on_js_received);


    ros::spin();

    return 0;
}
