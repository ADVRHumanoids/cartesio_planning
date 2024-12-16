#include "impl/planning_scene_wrapper.hxx"
#include "impl/utils.hxx"
#include "../impl/profiling.hxx"

#include <tf2_eigen/tf2_eigen.hpp>

using namespace XBot::Cartesian::Planning;
using namespace std::chrono_literals;

PlanningSceneWrapper::Impl::Impl(ModelInterface::ConstPtr model,
                                 rclcpp::Node::SharedPtr node):
    _model(model),
    _node(node ? node : rclcpp::Node::make_shared("planning_scene_wrapper_node"))
{
    // create sub-node for multi-threaded execution
    _node_mt = _node->create_sub_node("mt");
    auto cg_mt = _node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    _exe_mt.add_callback_group(cg_mt, _node->get_node_base_interface());

    // create robot model loader
    robot_model_loader::RobotModelLoader::Options rml_opt(_model->getUrdfString(),
                                                          _model->getSrdfString());

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(_node, rml_opt);


    // planning scene monitor automatically updates planning scene from topics
    _monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(_node, rml);

    // provide get planning scene server
    _monitor->providePlanningSceneService();

    // provide apply planning scene service
    _apply_planning_scene_srv
        = _node->create_service<moveit_msgs::srv::ApplyPlanningScene>(
            "apply_planning_scene",
            [this](moveit_msgs::srv::ApplyPlanningScene::Request::ConstSharedPtr req,
                   moveit_msgs::srv::ApplyPlanningScene::Response::SharedPtr res) {
                _monitor->newPlanningSceneMessage(req->scene);
                return true;
            },
            rclcpp::ServicesQoS(),
            cg_mt);

    // start async spinner
    _th_exit_flag = false;
    _th = std::make_unique<std::thread>(
        [this]()
        {
            while(!_th_exit_flag)
            {
                _exe_mt.spin_some(1s);
            }
        });
}

void PlanningSceneWrapper::Impl::update()
{
    // acquire lock for thread-safe access to the planning scene
    utils::MonitorLockguardWrite lock_w(_monitor); // RAII-style lock acquisition

    // retrieve robot state data struct
    auto& robot_state = _monitor->getPlanningScene()->getCurrentStateNonConst();

    // update planning scene from model
    for(const auto& jptr : _model->getJoints())
    {
        auto jname = jptr->getName();
        auto jtype = jptr->getType(); // joint type

        if(jtype == urdf::Joint::REVOLUTE  ||
            jtype == urdf::Joint::PRISMATIC ||
            jtype == urdf::Joint::CONTINUOUS)
        {
            robot_state.setJointPositions(jname, {jptr->getJointPositionMinimal().value()}); // joint value is a simple scalar
        }
        else if(jtype == urdf::Joint::FLOATING) // joint value is actually a pose (3 + 4 values)
        {
            auto jmodel = jptr->getUrdfJoint();

            std::string parent_link = jmodel->parent_link_name;
            std::string child_link = jmodel->child_link_name;

            // transform from parent link to child link
            Eigen::Affine3d p_T_c;
            _model->getPose(child_link, parent_link, p_T_c);

            // transform from parent link to joint predecessor frame
            Eigen::Affine3d p_T_j;
            p_T_j.setIdentity();
            p_T_j.translation().x() = jmodel->parent_to_joint_origin_transform.position.x;
            p_T_j.translation().y() = jmodel->parent_to_joint_origin_transform.position.y;
            p_T_j.translation().z() = jmodel->parent_to_joint_origin_transform.position.z;

            Eigen::Quaterniond p_q_j(
                jmodel->parent_to_joint_origin_transform.rotation.w,
                jmodel->parent_to_joint_origin_transform.rotation.x,
                jmodel->parent_to_joint_origin_transform.rotation.y,
                jmodel->parent_to_joint_origin_transform.rotation.z
                );

            p_T_j.linear() = p_q_j.toRotationMatrix();

            // joint transform
            Eigen::Affine3d Tj = p_T_j.inverse() * p_T_c;

            Eigen::Quaterniond Tj_rotation(Tj.linear());

            std::vector<double> jpos =
                {
                    Tj.translation().x(),
                    Tj.translation().y(),
                    Tj.translation().z(),
                    Tj_rotation.x(),
                    Tj_rotation.y(),
                    Tj_rotation.z(),
                    Tj_rotation.w()
                };

            robot_state.setJointPositions(jname, jpos);
            robot_state.update();

        }
        else if(jtype == urdf::Joint::FIXED)
        {
            // do nothing
        }
        else
        {
            throw std::runtime_error("Unsupported joint type");
        }

    }

    _monitor->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_STATE);
}

bool PlanningSceneWrapper::Impl::checkCollisions() const
{
    utils::MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;

    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkCollision(collision_request, collision_result);

    return collision_result.collision;
}

bool PlanningSceneWrapper::Impl::checkSelfCollisions() const
{
    utils::MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkSelfCollision(collision_request, collision_result);

    return collision_result.collision;
}

double PlanningSceneWrapper::Impl::computeCollisionDistance() const
{
    utils::MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_request.distance = true;

    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkCollision(collision_request, collision_result);

    return collision_result.distance;
}

double PlanningSceneWrapper::Impl::computeSelfCollisionDistance() const
{
    utils::MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_request.distance = true;

    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkSelfCollision(collision_request, collision_result);

    return collision_result.distance;
}

std::vector<std::string> PlanningSceneWrapper::Impl::getCollidingLinks() const
{
    utils::MonitorLockguardRead lock_r(_monitor);

    std::vector<std::string> links;
    _monitor->getPlanningScene()->getCollidingLinks(links);

    return links;
}

void PlanningSceneWrapper::Impl::applyPlanningScene(const moveit_msgs::msg::PlanningScene &scene)
{
    _monitor->updateFrameTransforms();
    _monitor->newPlanningSceneMessage(scene);
}

bool PlanningSceneWrapper::Impl::addCollisionObject(moveit_msgs::msg::CollisionObject co,
                                                    std::string attach_to_link,
                                                    std::vector<std::string> touch_links)
{
    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;

    // attached object
    if(!attach_to_link.empty())
    {
        moveit_msgs::msg::AttachedCollisionObject ao;
        ao.object = co;
        ao.link_name = attach_to_link;
        ao.touch_links = touch_links;

        ps.robot_state.is_diff = true;
        ps.robot_state.attached_collision_objects = {ao};
    }
    // world object
    else
    {
        ps.world.collision_objects.push_back(co);
    }

    applyPlanningScene(ps);

    return true;
}

bool PlanningSceneWrapper::Impl::addBox(std::string id,
                                        const Eigen::Vector3d &size,
                                        const Eigen::Affine3d &T,
                                        std::string frame_id,
                                        std::string attach_to_link,
                                        std::vector<std::string> touch_links)
{
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    co.pose = tf2::toMsg(T);

    shape_msgs::msg::SolidPrimitive solid;
    solid.type = solid.BOX;
    solid.dimensions = {size.x(), size.y(), size.z()};
    co.primitives.push_back(solid);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

bool PlanningSceneWrapper::Impl::addSphere(std::string id,
                                           double radius,
                                           const Eigen::Affine3d &T,
                                           std::string frame_id,
                                           std::string attach_to_link,
                                           std::vector<std::string> touch_links)
{
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    co.pose = tf2::toMsg(T);

    shape_msgs::msg::SolidPrimitive solid;
    solid.type = solid.SPHERE;
    solid.dimensions = {radius};
    co.primitives.push_back(solid);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

bool PlanningSceneWrapper::Impl::addCylinder(std::string id,
                                             double radius,
                                             double height,
                                             const Eigen::Affine3d &T,
                                             std::string frame_id,
                                             std::string attach_to_link,
                                             std::vector<std::string> touch_links)
{
    moveit_msgs::msg::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    co.pose = tf2::toMsg(T);

    shape_msgs::msg::SolidPrimitive solid;
    solid.type = solid.CYLINDER;
    solid.dimensions = {height, radius};
    co.primitives.push_back(solid);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

PlanningSceneWrapper::Impl::~Impl()
{
    _th_exit_flag = true;
    _th->join();
}



PlanningSceneWrapper::PlanningSceneWrapper(ModelInterface::ConstPtr model,
                                           rclcpp::Node::SharedPtr node)
{
    impl = std::make_unique<Impl>(model, node);
}

void PlanningSceneWrapper::update()
{
    return impl->update();
}

bool PlanningSceneWrapper::checkCollisions() const
{
    return impl->checkCollisions();
}

std::vector<std::string> PlanningSceneWrapper::getCollidingLinks() const
{
    return impl->getCollidingLinks();
}

PlanningSceneWrapper::~PlanningSceneWrapper()
{
}


PlanningSceneChecker::PlanningSceneChecker(PlanningSceneWrapper::Ptr ps,
                                           StateSpace::ConstPtr space,
                                           std::string name,
                                           int substate_idx):
    StateValidityChecker(space, name, substate_idx),
    _ps(ps)
{

}

bool XBot::Cartesian::Planning::PlanningSceneChecker::checkValid(const Eigen::VectorXd &q,
                                                                 std::optional<Eigen::VectorXd> &qnear) const
{
    TIKTOK(collision_check);

    _ps->update();

    return !_ps->checkCollisions();

}

void XBot::Cartesian::Planning::PlanningSceneChecker::printInvalidStateInformation(std::ostream &os) const
{
    auto colliding_links = _ps->getCollidingLinks();

    os << "[";

    for(const auto& c : colliding_links)
    {
        os << c << ", ";
    }

    os << "]";
}
