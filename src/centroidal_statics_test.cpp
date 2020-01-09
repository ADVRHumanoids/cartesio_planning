#include <sensor_msgs/JointState.h>
#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include "validity_checker/stability/centroidal_statics.h"

using namespace XBot::Cartesian::Planning;


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "centroidal_statics_test");
    ros::NodeHandle nh;

    auto cfg = XBot::ConfigOptionsFromParamServer();
    XBot::ModelInterface::Ptr model = XBot::ModelInterface::getModel(cfg);

    std::vector<std::string> links_in_contact;
    links_in_contact.push_back("l_sole");
    links_in_contact.push_back("r_sole");
    double mu = 0.5;
    CentroidalStatics cs(model, links_in_contact, mu, true);
    CentroidalStaticsROS csROS(model, cs, nh);

    auto on_js_received = [&csROS, model](const sensor_msgs::JointStateConstPtr& msg)
    {
        Eigen::VectorXd q(model->getJointNum()); q.setZero();
        for(int i = 0; i < msg->name.size(); i++)
            q[i] = msg->position[i];

        model->setJointPosition(q);
        model->update();

        csROS.publish();
    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("cartesian/solution", 1, on_js_received);

    ros::spin();

    return 0;
}