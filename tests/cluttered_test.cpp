#include <ros/ros.h>
#include <ros/service.h>
#include <thread>
#include <chrono>

#include <XBotInterface/ModelInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <matlogger2/matlogger2.h>

#include "validity_checker/validity_checker_context.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "ik/position_ik_solver.h"
#include "goal/NSPG.h"

#include <cartesio_planning/SetContactFrames.h>
#include <geometry_msgs/Quaternion.h>
#include <std_srvs/Empty.h>
#include <moveit_msgs/CollisionObject.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

XBot::ModelInterface::Ptr model;
std::vector<std::vector<Eigen::Affine3d>> stances;
std::vector<std::vector<std::string>> active_links;
std::shared_ptr<XBot::Cartesian::Planning::ValidityCheckContext> vc_context;
XBot::Cartesian::Planning::PositionCartesianSolver::Ptr solver;
std::vector<Eigen::VectorXd> qList;
std::string env = getenv("ROBOTOLOGY_ROOT");
XBot::MatLogger2::Ptr logger = XBot::MatLogger2::MakeLogger(env + "/external/cartesio_planning/log/cluttered_test");
std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> rspub;

ros::ServiceServer apply_planning_scene_srv, get_planning_scene_srv, start_srv;
ros::Publisher contact_pub, mesh_viz_pub;

Eigen::Matrix3d rotation(std::vector<double> normal)
{
    Eigen::Matrix3d Rx, Ry, Rz;
    std::vector<double> theta;
    
    if (normal[1] == -1.0)
    {
//         theta = {3.1415/2, 0.0, 0.0};
        theta = {0.0, 3*M_PI/4, 0.0};
    }
    
    if (normal[1] == 1.0)
    {       
//         theta = {-3.1415/2, 0.0, 0.0};
        theta = {0.0, 3*M_PI/4, 0.0};
    }
    
    Rx << 1.0, 0.0, 0.0, 0.0, cos(theta[0]) -sin(theta[0]), 0.0, sin(theta[0]), cos(theta[0]);
    Ry << cos(theta[1]), 0.0, sin(theta[1]), 0.0, 1.0, 0.0, -sin(theta[1]), 0.0, cos(theta[1]);
    Rz << cos(theta[2]), -sin(theta[2]), 0.0, sin(theta[2]), cos(theta[2]), 0.0, 0.0, 0.0, 1.0; 
    
    return Rx*Ry*Rz;
}

Eigen::Matrix3d generalRotation(std::vector<double> normal, std::string axis)
{
    Eigen::Vector3d n;
    n << normal[0], normal[1], normal[2];
    n.normalize();  
    
    Eigen::Vector3d x_child, y_child, z_child;
    Eigen::Vector3d x_parent ( 1, 0, 0 ), y_parent ( 0, 1, 0 ), z_parent ( 0, 0, 1 );
    Eigen::Matrix3d R;
    double value;    
    
    x_child = n;

    std::cout << "x_child norm: " << x_child.norm() << std::endl;
    
    // NEW METHOD
    value = x_child(0) * x_child(0) + x_child(1) * x_child(1);
    if (x_child(1) > 0 && x_child(2) > 0)
        z_child(2) = -std::sqrt(value);
    else
        z_child(2) = std::sqrt(value);
    std::cout << "z_child(2): " << z_child(2) << std::endl;
    
    double b = x_child(1) * x_child(2) * z_child(2);
    std::cout << "b: " << b << std::endl;
    double delta = pow(b,2) - pow(z_child(2),2)*(pow(x_child(2),2)*pow(z_child(2),2) + pow(x_child(0),2)*pow(z_child(2),2) - pow(x_child(0),2));
    std::cout << "delta before: " << delta << std::endl;
    if (delta < 0)
        delta = 0;
    std::cout << "delta: " << delta << std::endl;
    value = (b + sqrt(delta)) / pow(z_child(2),2);
    if ((x_child(1) < 0 && x_child(2) > 0) || (x_child(1) > 0 && x_child(2) > 0))
        z_child(1) = -value;
    else
        z_child(1) = value;
    
    if (x_child(0) == 0)
        z_child(0) = 0;
    else
        z_child(0) = (-x_child(2)*z_child(2) - x_child(1)*z_child(1)) / x_child(0);

    std::cout << "x_child: " << x_child.transpose() << std::endl;
    std::cout << "z_child: " << z_child.transpose() << std::endl;
    std::cout << "z_child norm: " << z_child.norm() << std::endl;
    z_child.normalize();
    std::cout << "z_child after: " << z_child.transpose() << std::endl;
    std::cout << "z_child norm after:  " << z_child.norm() << std::endl;

    std::cout << "y_child: " << y_child.transpose() << std::endl;
    std::cout << "y_child norm: " << y_child.norm() << std::endl;
    y_child = z_child.cross(x_child);
    y_child.normalize();
    std::cout << "y_child after: " << y_child.transpose() << std::endl;
    std::cout << "y_child norm after: " << y_child.norm() << std::endl;


    // Create the rotation matrix between parent and child frames
    R << x_child.transpose() * x_parent, y_child.transpose() * x_parent, z_child.transpose() * x_parent,
         x_child.transpose() * y_parent, y_child.transpose() * y_parent, z_child.transpose() * y_parent,
         x_child.transpose() * z_parent, y_child.transpose() * z_parent, z_child.transpose() * z_parent;


    std::cout << "det(R): " << R.determinant() << std::endl;
    std::cout << "R'*R: " << R.transpose()*R << std::endl;
    
    if (axis == "z")
    {
        Eigen::Matrix3d Ry;
        Ry << cos(M_PI/2), 0.0, sin(M_PI/2), 0.0, 1.0, 0.0, -sin(M_PI/2), 0.0, cos(M_PI/2);
        return R*Ry;
    }
    else if (axis == "y")
    {
        Eigen::Matrix3d Rz;
        Rz << cos(M_PI/2), -sin(M_PI/2), 0.0, sin(M_PI/2), cos(M_PI/2), 0.0, 0.0, 0.0, 1.0; 
        return R*Rz;
    }
    else if (axis == "-z")
    {
        Eigen::Matrix3d Ry;
        Ry << cos(-M_PI/2), 0.0, sin(-M_PI/2), 0.0, 1.0, 0.0, -sin(-M_PI/2), 0.0, cos(-M_PI/2);
        return R*Ry;
    }
    return R;
}

void generateStances()
{
    if (stances.size() > 0)
        stances.clear();
    if (active_links.size() > 0)
        active_links.clear();
    
    Eigen::Affine3d TLFoot, TRFoot, TLHand, TRHand;
//     std::vector<double> n_R_h = {0.0, 1.0, 0.0};
//     std::vector<double> n_L_h = {0.0, -1.0, 0.0};
    std::vector<double> n_R_h = {-1/sqrt(2), 0.0, 1/sqrt(2)};
    std::vector<double> n_L_h = {-1/sqrt(2), 0.0, 1/sqrt(2)};
    std::vector<double> n_R_f = {0.0, 0.0995, 0.9950};
    std::vector<double> n_L_f = {0.0, -0.0995, 0.9950};
    
    // stance 0
    TLFoot.translation() << 0.0, 0.1, 0.0;
    TLFoot.linear() = Eigen::Matrix3d::Identity();
    TRFoot.translation() << 0.0, -0.1, 0.0;
    TRFoot.linear() = Eigen::Matrix3d::Identity();    
    active_links.push_back({"l_sole", "r_sole"});
    stances.push_back({TLFoot, TRFoot});
    
    // stance 1
    TRHand.translation() << 0.5, -0.3, 1.0;
    TRHand.linear() = generalRotation(n_R_h, "-z");   
    active_links.push_back({"l_sole", "r_sole", "TCP_R"});
    stances.push_back({TLFoot, TRFoot, TRHand});
    
    // stance 2
    active_links.push_back({"l_sole", "TCP_R"});
    stances.push_back({TLFoot, TRHand});
    
    // stance 3
    TRFoot.translation() << 0.25, -0.1, 0.2;
    active_links.push_back({"l_sole", "r_sole", "TCP_R"});
    stances.push_back({TLFoot, TRFoot, TRHand});
    
    // stance 4
    active_links.push_back({"r_sole", "TCP_R"});
    stances.push_back({TRFoot, TRHand});
    
    // stance 5
    TLFoot.translation() << 0.25, 0.1, 0.2;
    active_links.push_back({"l_sole", "r_sole", "TCP_R"});
    stances.push_back({TLFoot, TRFoot, TRHand});
    
    // stance 6
    active_links.push_back({"l_sole","r_sole"});
    stances.push_back({TLFoot, TRFoot});
    
    // stance 7
    TLHand.translation() << 0.7, 0.3, 1.2;
    TLHand.linear() = generalRotation(n_L_h, "-z");
    active_links.push_back({"l_sole", "r_sole", "TCP_L"});
    stances.push_back({TLFoot, TRFoot, TLHand});
    
    //stance 8
    active_links.push_back({"r_sole", "TCP_L"});
    stances.push_back({TRFoot, TLHand});
    
    // stance 9
    TLFoot.translation() << 0.5, 0.1, 0.4;
    active_links.push_back({"l_sole", "r_sole", "TCP_L"});
    stances.push_back({TLFoot, TRFoot, TLHand});
    
    // stance 10
    active_links.push_back({"l_sole", "TCP_L"});
    stances.push_back({TLFoot, TLHand});
    
    // stance 11
    TRFoot.translation() << 0.5, -0.1, 0.4;
    active_links.push_back({"l_sole", "r_sole", "TCP_L"});
    stances.push_back({TLFoot, TRFoot, TLHand});
    
    // stance 12
    active_links.push_back({"l_sole", "r_sole"});
    stances.push_back({TLFoot, TRFoot});

    // stance 13
    active_links.push_back({"l_sole"});
    stances.push_back({TLFoot});

    // stance 14
    TRFoot.translation() << 0.8, -0.1, 0.4;
    TRFoot.linear() = generalRotation(n_R_f, "z");
    active_links.push_back({"l_sole", "r_sole"});
    stances.push_back({TLFoot, TRFoot});

    // stance 15
    TLHand.translation() << 0.8, 0.3, 1.6;
    TLHand.linear() = generalRotation({0.0, -1/sqrt(2), 1/sqrt(2)}, "-z");
    active_links.push_back({"l_sole", "r_sole", "TCP_L"});
    stances.push_back({TLFoot, TRFoot, TLHand});

    // stance 16
    active_links.push_back({"r_sole", "TCP_L"});
    stances.push_back({TRFoot, TLHand});

    // stance 17
    TLFoot.translation() << 0.8, 0.1, 0.4;
    TLFoot.linear() = generalRotation(n_L_f, "z");
    active_links.push_back({"l_sole", "r_sole", "TCP_L"});
    stances.push_back({TLFoot, TRFoot, TLHand});

    // stance 18
    active_links.push_back({"l_sole", "r_sole"});
    stances.push_back({TLFoot, TRFoot});
}

void loadModel()
{
    auto cfg = XBot::ConfigOptionsFromParamServer();
    model = XBot::ModelInterface::getModel(cfg);
}

bool apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request& req, moveit_msgs::ApplyPlanningScene::Response& res) 
{
    vc_context->planning_scene->applyPlanningScene(req.scene);
    return true;
}

bool get_planning_scene_service(moveit_msgs::GetPlanningScene::Request& req, moveit_msgs::GetPlanningScene::Response& res)
{
    vc_context->planning_scene->getPlanningScene(req, res);
    return true;
}

void loadValidityChecker(ros::NodeHandle nh, YAML::Node planner_config)
{
    vc_context = std::make_shared<XBot::Cartesian::Planning::ValidityCheckContext>(planner_config, model, nh);
    
    vc_context->planning_scene->startMonitor();
    vc_context->planning_scene->startMonitor();
    
    get_planning_scene_srv = nh.advertiseService<moveit_msgs::GetPlanningScene::Request, moveit_msgs::GetPlanningScene::Response>("planner/get_planning_scene", &get_planning_scene_service);
    apply_planning_scene_srv = nh.advertiseService<moveit_msgs::ApplyPlanningScene::Request, moveit_msgs::ApplyPlanningScene::Response>("planner/apply_planning_scene", &apply_planning_scene_service); 
}

void generateConfigurations(std::vector<std::vector<Eigen::Affine3d>> stances, 
                                                    std::vector<std::vector<std::string>> active_links,
                                                    std::vector<Eigen::VectorXd>& qList
                                                   )
{
    if (stances.size() != active_links.size())
    {
        ROS_ERROR("stances and active_links vector has different size!");
    }
    
    if (qList.size() > 0)
        qList.clear();
    
    std::vector<std::string> all_tasks{"l_sole", "r_sole", "TCP_R", "TCP_L"};
    auto NSPG = std::make_shared<XBot::Cartesian::Planning::NSPG>(solver, *vc_context); 
    
    cartesio_planning::SetContactFrames contacts;   
    for (int i = 0; i < stances.size(); i++)
    {
        std::cout << "processing stance " << i << std::endl;
        
        for (auto task : all_tasks)
        {
            std::vector<std::string>::iterator it = std::find(active_links[i].begin(), active_links[i].end(), task);
            if (it == active_links[i].end())
                solver->getCI()->getTask(task)->setActivationState(XBot::Cartesian::ActivationState::Disabled);
            else
                solver->getCI()->getTask(task)->setActivationState(XBot::Cartesian::ActivationState::Enabled);
        }
        
        for (int j = 0; j < stances[i].size(); j++)
        {
            solver->setDesiredPose(active_links[i][j], stances[i][j]);
        }
        
        contacts.action = cartesio_planning::SetContactFrames::SET;
        contacts.frames_in_contact = active_links[i];
        
        std::vector<geometry_msgs::Quaternion> rot;
        for (int j = 0; j < active_links[i].size(); j++)
        {
            geometry_msgs::Quaternion quat;
            Eigen::Matrix3d Rx;
            Rx << 1, 0, 0, 0, cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI);
            if (active_links[i][j] == "TCP_R" || active_links[i][j] == "TCP_L")
            {
                Eigen::Matrix3d R = stances[i][j].linear();
                tf::quaternionEigenToMsg(Eigen::Quaternion<double>(R*Rx), quat);
            }
            else
                tf::quaternionEigenToMsg(Eigen::Quaternion<double>(stances[i][j].linear()), quat);
            rot.push_back(quat);
        }
        contacts.rotations = rot;
        
        if (active_links[i].size() <= 2)
            contacts.optimize_torque = true;
        else
            contacts.optimize_torque = false;
        
        contacts.friction_coefficient = 0.72;
        
        contact_pub.publish(contacts);
        ros::spinOnce();       
        
        if (!qList.empty())
        {
            model->setJointPosition(qList.back());
            model->update();
        }
        
        Eigen::VectorXd qhome;
        NSPG->getIKSolver()->getModel()->getRobotState("home", qhome);
        NSPG->getIKSolver()->getModel()->setJointPosition(qhome);
        NSPG->getIKSolver()->getModel()->update();
        auto tic = std::chrono::high_resolution_clock::now();
//        if (qList.size() > 0)
//        {
//            NSPG->getIKSolver()->getModel()->setJointPosition(qList[qList.size()-1]);
//            NSPG->getIKSolver()->getModel()->update();
//        }
        
        if (!NSPG->sample(3.0))
        {
            auto toc = std::chrono::high_resolution_clock::now();
            rspub->publishTransforms(ros::Time::now(), "");
            std::chrono::duration<float> fsec = toc - tic;
            logger->add("success", 0);
            logger->add("time", fsec.count());
            std::cout << "[NSPG]: unable to find a feasible solution!" << std::endl;
        }
        else
        {
            auto toc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<float> fsec = toc - tic;
            logger->add("success", 1);
            logger->add("time", fsec.count());
            std::cout << "[NSPG]: solution found!" << std::endl;
            Eigen::VectorXd q(model->getJointNum());
            NSPG->getIKSolver()->getModel()->getJointPosition(q);
            qList.push_back(q);
        }           
    }
    
    std::cout << "All stances processed!" << std::endl;
    std::cout << "Computed " << qList.size() << "/" << stances.size() << " configurations" << std::endl;
    NSPG->_logger.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    
}

bool start_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    generateConfigurations(stances, active_links, qList);
    logger.reset();
    return true;
}

bool normal_test_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    Eigen::Affine3d T, Tdes;
    model->getPose("TCP_R", T);
    
    Tdes = T;
//    Tdes.translation().z() += 0.5;
//     Tdes.linear() = generalRotation({T.linear()(0,2), T.linear()(1,2), T.linear()(2,2)}, "-z");
    
    Tdes.translation() << 0.0, 0.0, 0.0;
    std::vector<double> n = {0.0, 0.0995, 0.9950};
    Tdes.linear() = generalRotation(n, "x");
    
    tf::Transform transform;

    // Broadcast the transformation
    tf::transformEigenToTF ( Tdes, transform );
//     transform.getRotation().normalize();
    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform ( tf::StampedTransform ( transform, ros::Time::now(), "world", "normal_test" ) );
    
    return true;
}

void setTiles()
{
    if (stances.size() != active_links.size())
        ROS_ERROR("stances and active_links have different sizes");
    
    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "world";
    obj.header.stamp = ros::Time::now();
    obj.id = "tiles";
    
    for (int i = 0; i < stances.size(); i++)
    {
        for (int j = 0; j < stances[i].size(); j++)
        {
            shape_msgs::SolidPrimitive primitive;
            primitive.type = shape_msgs::SolidPrimitive::BOX;
            primitive.dimensions = {0.2, 0.2, 0.01};
            obj.primitives.push_back(primitive);
            
            geometry_msgs::Pose pose;
            pose.position.x = stances[i][j].translation().x(); 
            pose.position.y = stances[i][j].translation().y();
            pose.position.z = stances[i][j].translation().z();
            tf::quaternionEigenToMsg(Eigen::Quaternion<double>(stances[i][j].linear()), pose.orientation);
            obj.primitive_poses.push_back(pose);
        }
    }
    
    obj.operation = moveit_msgs::CollisionObject::ADD;
    
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.world.collision_objects.push_back(obj);
    vc_context->planning_scene->applyPlanningScene(planning_scene);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluttered_test");
    ros::NodeHandle nh; 

    
    contact_pub = nh.advertise<cartesio_planning::SetContactFrames>("contacts", 10, true);
    start_srv = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("start_service", &start_service);
    auto normal_test_srv = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>("normal_test", &normal_test_service);
    mesh_viz_pub = nh.advertise<moveit_msgs::CollisionObject>("planner/collision_objects", 10, true);
    
    
    loadModel();
    generateStances();
    
    
    for (int i = 0; i < stances.size(); i ++)
    {
        std::cout << "STANCE " << i << std::endl;
        for (int j = 0; j < stances[i].size(); j++)
        {
            std::cout << "link: " << active_links[i][j] << std::endl;
            std::cout << stances[i][j].matrix() << std::endl;
        }
    }
    
    std::string planner_config_string;
    nh.getParam("planner_config", planner_config_string);
    std::cout << "Loaded config file: \n" << planner_config_string << std::endl;
    
    std::string problem_description;
    if(!nh.getParam("problem_description", problem_description))
    {
        ROS_ERROR("problem_description!");
        throw std::runtime_error("problem_description!");
    }

    auto ik_yaml_goal = YAML::Load(problem_description);

    double ci_period = 0.1;
    auto ci_ctx = std::make_shared<XBot::Cartesian::Context>(std::make_shared<XBot::Cartesian::Parameters>(ci_period), model);
    auto ik_prob = XBot::Cartesian::ProblemDescription(ik_yaml_goal, ci_ctx);

    auto ci = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot", ik_prob, ci_ctx);     
    
    solver = std::make_shared<XBot::Cartesian::Planning::PositionCartesianSolver>(ci);
    
    Eigen::VectorXd qhome(model->getJointNum());
    model->getRobotState("home", qhome);
    XBot::JointNameMap jmap;
    model->eigenToMap(qhome, jmap);
    solver->getCI()->setReferencePosture(jmap);
    model->setJointPosition(qhome);
    model->update();

    YAML::Node planner_config = YAML::Load(planner_config_string);
    loadValidityChecker(nh, planner_config);
    vc_context->planning_scene->acm.setEntry("LFoot", "tiles", true);
    vc_context->planning_scene->acm.setEntry("RFoot", "tiles", true);
    vc_context->planning_scene->acm.setEntry("RBall", "tiles", true);
    vc_context->planning_scene->acm.setEntry("LBall", "tiles", true);
    setTiles();
    
    vc_context->vc_aggregate.checkAll();
    
    rspub = std::make_shared<XBot::Cartesian::Utils::RobotStatePublisher>(model); 
    
    ros::Rate rate(1);
    while (ros::ok())
    {
        if (qList.size() > 0)
        {
            for (auto q : qList)
            {
                model->setJointPosition(q);
                model->update();
                rspub->publishTransforms(ros::Time::now(), "");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        else
        { 
            rspub->publishTransforms(ros::Time::now(), "");
            ros::spinOnce();
        }
        rate.sleep();
    }
    
    return 0;
}
