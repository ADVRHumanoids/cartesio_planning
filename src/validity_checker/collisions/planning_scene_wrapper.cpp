#include <cartesio_planning/validity_checker/planning_scene_wrapper.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/crop_box.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>


namespace
{

/**
 * @brief The MonitorLockguardWrite class provides a RAII-style read-write lock
 * for the planning scene monitor. Constructing an object will acquire the lock,
 * which will automatically be released when the locker goes out of scope.
 * See also std::lock_guard<>.
 */
class MonitorLockguardWrite
{

public:

    MonitorLockguardWrite(planning_scene_monitor::PlanningSceneMonitorPtr monitor)
    {
        _monitor = monitor;
        _monitor->lockSceneWrite();
    }

    ~MonitorLockguardWrite()
    {
        _monitor->unlockSceneWrite();
    }

private:

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;
};


/**
 * @brief The MonitorLockguardRead class provides a RAII-style read-only lock
 * for the planning scene monitor. Constructing an object will acquire the lock,
 * which will automatically be released when the locker goes out of scope.
 * See also std::lock_guard<>.
 */
class MonitorLockguardRead
{

public:

    MonitorLockguardRead(planning_scene_monitor::PlanningSceneMonitorPtr monitor)
    {
        _monitor = monitor;
        _monitor->lockSceneRead();
    }

    ~MonitorLockguardRead()
    {
        _monitor->unlockSceneRead();
    }

private:

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;
};
}

namespace XBot { namespace Cartesian { namespace Planning {

PlanningSceneWrapper::PlanningSceneWrapper(ModelInterface::ConstPtr model):
    _model(model),
    _async_spinner(1, &_queue)
{
    // create robot model loader
    robot_model_loader::RobotModelLoader::Options rml_opt(_model->getUrdfString(),
                                                          _model->getSrdfString());

    auto rml = std::make_shared<robot_model_loader::RobotModelLoader>(rml_opt);


    // planning scene monitor automatically updates planning scene from topics
    _monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(rml);

    _srdf = _model->getSrdf();

    computeChainToLinks();
}

void PlanningSceneWrapper::startMonitor()
{
    // publish planning scene at 30 Hz (topic is ~/monitored_planning_scene)
    _monitor->setPlanningScenePublishingFrequency(20.); // tbd: hardcoded

    // this subscribes to /planning_scene
    _monitor->startSceneMonitor();

    // this is somehow different from the scene monitor.. boh
    //    _monitor->startWorldGeometryMonitor();

    // this starts monitored planning scene publisher
    _monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    
    // AllowedCollisionMatrix definition. Entries can be added anywhere in the code simply
    // with acm.setEntry(std::string name1, std::string name2, bool allowed)
    acm = _monitor->getPlanningScene()->getAllowedCollisionMatrix();

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&_queue);
    _apply_planning_scene_srv = nh.advertiseService("apply_planning_scene_service", &PlanningSceneWrapper::apply_planning_scene_service, this);
}

void PlanningSceneWrapper::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg, int i)
{
    if(!_point_clouds[i])
    {
        _point_clouds[i].reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
    *pc = *msg;

    std::lock_guard<std::mutex> lg(_pc_mtx);

    transform_point_cloud(pc, _point_clouds[i], "base_link");
}

bool PlanningSceneWrapper::apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request& req, moveit_msgs::ApplyPlanningScene::Response& res)
{
    applyPlanningScene(req.scene);
    return true;
}

void PlanningSceneWrapper::stopMonitor()
{
    _monitor->stopSceneMonitor();
    _monitor->stopWorldGeometryMonitor();
    _monitor->stopPublishingPlanningScene();
}

void PlanningSceneWrapper::startGetPlanningSceneServer()
{
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&_queue);

//    _get_ps_srv = nh.advertiseService("get_planning_scene",
//                                      &PlanningSceneWrapper::getPlanningScene,
//                                      this);

    _monitor->providePlanningSceneService();

    _async_spinner.start();

}

void PlanningSceneWrapper::startOctomapServer(std::vector<std::string> input_topics)
{
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&_queue);

    _point_clouds.resize(input_topics.size());

    int i = 0;

    for(auto topic: input_topics)
    {
        std::cout << "startOctomapServer: subscribed to " << topic << "\n";

        auto cb = [this, i](const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
        {
            pc_callback(msg, i);
        };

        auto sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, cb);

        _pc_subs.push_back(sub);

        i++;
    }

    _add_octomap_srv = nh.advertiseService("octomap_service", &PlanningSceneWrapper::octomap_service, this);
}

void PlanningSceneWrapper::transform_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::string frame_id)
{
    tf::StampedTransform transform;
    try
    {
        _listener.waitForTransform(frame_id, cloud_in->header.frame_id, ros::Time(0), ros::Duration(10.0));
        _listener.lookupTransform(frame_id, cloud_in->header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Affine3d b_T_cam;
    tf::transformTFToEigen(transform, b_T_cam);
    pcl::transformPointCloud(*cloud_in, *cloud_out, b_T_cam.matrix());

    cloud_out->header.frame_id = frame_id;
}

bool PlanningSceneWrapper::octomap_service(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    res.success = updateOctomap();

    return true;
}

bool PlanningSceneWrapper::updateOctomap()
{
    std::lock_guard<std::mutex> lg(_pc_mtx);

    double resolution = 0.05;

    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(resolution);
    octomap::OcTree final_octree(resolution);
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxel_centers;
    
    for(auto pc : _point_clouds)
    {
        if(!pc)
        {
            continue;
        }

        // filter PointCloud to ignore nearest points (assuming they belong to the robot)
        // TODO: add robot_body_filtering somehow
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-0.7, -0.4, -1.0, 1));
        boxFilter.setMax(Eigen::Vector4f(0.7, 0.4, 2.0, 1));
        boxFilter.setNegative(true);
        boxFilter.setInputCloud(pc);
        boxFilter.filter(*pc);

        octree.setInputCloud(pc);
        octree.addPointsFromInputCloud();
        octree.getOccupiedVoxelCenters(voxel_centers);

        for (const auto & voxel_center: voxel_centers) {
            final_octree.updateNode(voxel_center.x, voxel_center.y, voxel_center.z, true, false);
        }

    }

    final_octree.updateInnerOccupancy();

    octomap_msgs::Octomap octomap;
    octomap_msgs::binaryMapToMsg(final_octree, octomap);

    octomap_msgs::OctomapWithPose octomap_with_pose;
    octomap_with_pose.header.frame_id = "base_link";
    octomap_with_pose.header.stamp = ros::Time::now();
    octomap_with_pose.octomap = octomap;
    moveit_msgs::PlanningScene ps;
    ps.world.octomap = octomap_with_pose;

    ps.is_diff = true;

    applyPlanningScene(ps);

    return true;
}

void PlanningSceneWrapper::clearOctomap()
{
    _monitor->clearOctomap();
}

bool PlanningSceneWrapper::updateOctomapFromTopic(std::string pc_topic,
                                                  double resolution,
                                                  double ground_height,
                                                  Eigen::Vector3d local_min,
                                                  Eigen::Vector3d local_max,
                                                  Eigen::Vector3d base_min,
                                                  Eigen::Vector3d base_max)
{
    auto pc_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pc_topic, ros::Duration(5.0));

    if(!pc_msg)
    {
        std::cerr << "no message received on topic " << pc_topic << " within timeout \n";
        return false;
    }

    // convert ros msg to pcl
    auto pc = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*pc_msg, *pc);

    // apply local filter if any
    if(!local_min.isZero() || !local_max.isZero())
    {
        Eigen::Vector4f boxMin, boxMax;
        boxMin << local_min.cast<float>(), 1;
        boxMax << local_max.cast<float>(), 1;

        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(boxMin);
        boxFilter.setMax(boxMax);
        boxFilter.setNegative(true);
        boxFilter.setInputCloud(pc);
        boxFilter.filter(*pc);
    }

    // transform to base link
    transform_point_cloud(pc, pc, "base_link");

    // apply global filter if any
    if(!base_min.isZero() || !base_max.isZero())
    {
        Eigen::Vector4f boxMin, boxMax;
        boxMin << base_min.cast<float>(), 1;
        boxMax << base_max.cast<float>(), 1;

        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(boxMin);
        boxFilter.setMax(boxMax);
        boxFilter.setNegative(true);
        boxFilter.setInputCloud(pc);
        boxFilter.filter(*pc);
    }

    // apply ground filter
    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-5., -5., ground_height, 1.0));
    boxFilter.setMax(Eigen::Vector4f(5., 5., ground_height + 3.0, 1.0));
    boxFilter.setNegative(false);
    boxFilter.setInputCloud(pc);
    boxFilter.filter(*pc);

    // get initial octomap (we're going to add this point cloud on top)
    moveit_msgs::GetPlanningSceneRequest ps_req;
    moveit_msgs::GetPlanningSceneResponse ps_res;
    ps_req.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    getPlanningScene(ps_req, ps_res);

    auto initial_octree_abstract = octomap_msgs::msgToMap(ps_res.scene.world.octomap.octomap);
    octomap::OcTree* initial_octree = nullptr;

    if(initial_octree_abstract)
    {
        initial_octree = dynamic_cast<octomap::OcTree*>(initial_octree_abstract);
    }
    else{
        initial_octree = new octomap::OcTree(resolution);
    }

    // to octree
    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZ> octree(resolution);
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > voxel_centers;

    octree.setInputCloud(pc);
    octree.addPointsFromInputCloud();
    octree.getOccupiedVoxelCenters(voxel_centers);

    for (const auto & vc: voxel_centers)
    {
        initial_octree->updateNode(vc.x, vc.y, vc.z, true, false);
    }

    initial_octree->updateInnerOccupancy();

    octomap_msgs::Octomap octomap;
    octomap_msgs::binaryMapToMsg(*initial_octree, octomap);

    octomap_msgs::OctomapWithPose octomap_with_pose;
    octomap_with_pose.header.frame_id = "base_link";
    octomap_with_pose.header.stamp = ros::Time::now();
    octomap_with_pose.octomap = octomap;
    moveit_msgs::PlanningScene ps;
    ps.world.octomap = octomap_with_pose;

    ps.is_diff = true;

    applyPlanningScene(ps);

    delete initial_octree;

    return true;
}

void PlanningSceneWrapper::update()
{
    // acquire lock for thread-safe access to the planning scene
    MonitorLockguardWrite lock_w(_monitor); // RAII-style lock acquisition

    // retrieve robot state data struct
    auto& robot_state = _monitor->getPlanningScene()->getCurrentStateNonConst();

    // retrieve modelinterface state
    XBot::JointNameMap q;
    _model->getJointPosition(q);

    // update planning scene from model
    for(const auto& jpair : _model->getUrdf().joints_)
    {
        auto jname = jpair.first; // joint name
        auto jmodel = jpair.second; // urdf::Joint model
        auto jtype = jmodel->type; // joint type

        if(jtype == urdf::Joint::REVOLUTE  ||
                jtype == urdf::Joint::PRISMATIC ||
                jtype == urdf::Joint::CONTINUOUS)
        {
            robot_state.setJointPositions(jname, {q.at(jname)}); // joint value is a simple scalar
        }
        else if(jtype == urdf::Joint::FLOATING) // joint value is actually a pose (3 + 4 values)
        {
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

bool PlanningSceneWrapper::checkCollisions() const
{
    MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;

    collision_detection::CollisionResult collision_result;  
    
    _monitor->getPlanningScene()->checkCollision(collision_request, collision_result);

    return collision_result.collision;
}

bool PlanningSceneWrapper::checkSelfCollisions() const
{
    MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkSelfCollision(collision_request, collision_result);

    return collision_result.collision;
}

std::vector<std::string> PlanningSceneWrapper::getCollidingLinks() const
{
    MonitorLockguardRead lock_r(_monitor);

    std::vector<std::string> links;
    _monitor->getPlanningScene()->getCollidingLinks(links);

    return links;
}

std::vector<XBot::ModelChain> PlanningSceneWrapper::getCollidingChains() const 
{
    std::vector<std::string> colliding_links = getCollidingLinks();
    std::set<std::string> colliding_chain_names;

    for(auto cl : colliding_links)
    {
        try
        {
            auto cname = _link_to_chain.at(cl);
            colliding_chain_names.insert(cname);
        }
        catch(std::out_of_range& e)
        {
            std::cerr << "[PlanningSceneWrapper::getCollidingChains] could not find chain for link " << cl << "\n";
        }
    }

    std::vector<XBot::ModelChain> colliding_chains;
    for(auto cc : colliding_chain_names)
    {
        colliding_chains.push_back(_model->chain(cc));
    }

    return colliding_chains;
}

void PlanningSceneWrapper::setPadding(double padding)
{
    MonitorLockguardWrite lock_rw(_monitor);

    _monitor->getPlanningScene()->getCollisionEnvNonConst()->setPadding(padding);
    _monitor->getPlanningScene()->propogateRobotPadding();

}


void PlanningSceneWrapper::applyPlanningScene(const moveit_msgs::PlanningScene & scene)
{
    _monitor->updateFrameTransforms();
    _monitor->newPlanningSceneMessage(scene);
}

bool PlanningSceneWrapper::addCollisionObject(moveit_msgs::CollisionObject co,
                                              std::string attach_to_link,
                                              std::vector<std::string> touch_links)
{
    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;

    // attached object
    if(!attach_to_link.empty())
    {
        moveit_msgs::AttachedCollisionObject ao;
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

bool PlanningSceneWrapper::addBox(std::string id,
                                  const Eigen::Vector3d &size,
                                  const Eigen::Affine3d &T,
                                  std::string frame_id,
                                  std::string attach_to_link,
                                  std::vector<std::string> touch_links)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    tf::poseEigenToMsg(T, co.pose);

    shape_msgs::SolidPrimitive solid;
    solid.type = solid.BOX;
    solid.dimensions = {size.x(), size.y(), size.z()};
    co.primitives.push_back(solid);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

bool PlanningSceneWrapper::addSphere(std::string id,
                                     double radius,
                                     const Eigen::Affine3d &T,
                                     std::string frame_id,
                                     std::string attach_to_link,
                                     std::vector<std::string> touch_links)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    tf::poseEigenToMsg(T, co.pose);

    shape_msgs::SolidPrimitive solid;
    solid.type = solid.SPHERE;
    solid.dimensions = {radius};
    co.primitives.push_back(solid);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

bool PlanningSceneWrapper::addCylinder(std::string id,
                                       double radius,
                                       double height,
                                       const Eigen::Affine3d &T,
                                       std::string frame_id,
                                       std::string attach_to_link,
                                       std::vector<std::string> touch_links)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.header.frame_id = frame_id;
    tf::poseEigenToMsg(T, co.pose);

    shape_msgs::SolidPrimitive solid;
    solid.type = solid.CYLINDER;
    solid.dimensions = {height, radius};
    co.primitives.push_back(solid);

    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    return addCollisionObject(co, attach_to_link, touch_links);
}

bool PlanningSceneWrapper::getPlanningScene(moveit_msgs::GetPlanningScene::Request & req,
                                            moveit_msgs::GetPlanningScene::Response & res)
{
    if (req.components.components & moveit_msgs::PlanningSceneComponents::TRANSFORMS)
    {
        _monitor->updateFrameTransforms();
    }

    planning_scene_monitor::LockedPlanningSceneRO ps(_monitor);

    moveit_msgs::PlanningSceneComponents all_components;
    all_components.components = UINT_MAX;  // Return all scene components if nothing is specified.
    ps->getPlanningSceneMsg(res.scene, req.components.components ? req.components : all_components);

    return true;

}

void PlanningSceneWrapper::computeChainToLinks()
{
    const auto& urdf = _model->getUrdf();

    auto chain_names = _model->getChainNames();

    for(auto ch : chain_names)
    {
        if(ch == "virtual_chain")
        {
            continue;
        }

        auto base_name = _model->chain(ch).getBaseLinkName();

        auto link_name = _model->chain(ch).getTipLinkName();
        auto link = urdf.getLink(link_name);

        std::cout << "[" << ch << "] started traversal from tip link " << link_name << "\n";
        std::set<std::string> links = {link_name};
        _link_to_chain[link_name] = ch;

        while(link->parent_joint)
        {
            link_name = link->parent_joint->parent_link_name;
            links.insert(link_name);
            std::cout << "[" << ch << "] adding link " << link_name << "\n";

            _link_to_chain[link_name] = ch;

            if(link_name == base_name)
            {
                break;
            }

            link = urdf.getLink(link_name);
        }

        if(link_name != base_name)
        {
            throw std::runtime_error("invalid base link name for chain '" + ch + "'");
        }

        _chain_to_links[ch] = links;
    }

    // complete mapping with all fixed links
    std::vector<urdf::LinkSharedPtr> urdf_links;
    urdf.getLinks(urdf_links);
    for(auto link : urdf_links)
    {
        // link already present, skip
        if(_link_to_chain.count(link->name) > 0)
        {
            continue;
        }

        // don't check root link
        if(!link->parent_joint)
        {
            continue;
        }

        // go up the tree
        urdf::LinkConstSharedPtr current_link = link;

        bool ok = false;

        while(!ok && current_link->parent_joint)

        {
            // std::cout << "trying to connect link " << current_link->name << ".. \n";

            // hack: skip base link
            if(current_link->name == "base_link")
            {
                ok = true;
                break;
            }

            current_link = urdf.getLink(current_link->parent_joint->parent_link_name);

            // known link found, add link to its chain
            if(_link_to_chain.count(current_link->name) > 0)
            {
                std::string ch = _link_to_chain.at(current_link->name);
                _link_to_chain[link->name] = ch;
                _chain_to_links.at(ch).insert(link->name);
                ok = true;
            }
        }

        // failure
        if(!ok)
        {
            throw std::runtime_error("unable to find chain for link '" + link->name + "'");
        }

    }

}


} } }
