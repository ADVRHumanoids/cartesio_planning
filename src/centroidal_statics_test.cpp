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

        cartesio_planning::SetContactFrames::Ptr msg2 = boost::make_shared<cartesio_planning::SetContactFrames>();
        msg2->action = msg2->SET;

        msg2->frames_in_contact.push_back("l_sole");
        msg2->frames_in_contact.push_back("r_sole");

        Eigen::Affine3d T;
        geometry_msgs::Pose P;
        model->getPose("l_sole", T);
        tf::poseEigenToMsg(T, P);
        msg2->rotations.push_back(P.orientation);

        model->getPose("r_sole", T);
        tf::poseEigenToMsg(T, P);
        msg2->rotations.push_back(P.orientation);

        msg2->friction_coefficient = 0.5;

        csROS.set_contacts(boost::static_pointer_cast<cartesio_planning::SetContactFrames>(msg2));

        csROS.publish();
    };

    auto js_sub = nh.subscribe<sensor_msgs::JointState>("cartesian/solution", 1, on_js_received);


    ros::spin();

    return 0;
}
