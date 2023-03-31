#include <cartesio_planning/validity_checker/planning_scene_wrapper.h>


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

    _get_ps_srv = nh.advertiseService("get_planning_scene",
                                      &PlanningSceneWrapper::getPlanningScene,
                                      this);

    _async_spinner.start();

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

//     std::cout << "colliding links: " << std::endl;
//     for (auto i : getCollidingLinks())
//         std::cout << i<< std::endl;

    return collision_result.collision;
}

bool PlanningSceneWrapper::checkSelfCollisions() const
{
    MonitorLockguardRead lock_r(_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    _monitor->getPlanningScene()->checkSelfCollision(collision_request, collision_result);

//         std::cout << "colliding links: " << std::endl;
//         for (auto i : getCollidingLinks())
//             std::cout << i<< std::endl;

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


void PlanningSceneWrapper::applyPlanningScene(const moveit_msgs::PlanningScene & scene)
{
    _monitor->updateFrameTransforms();
    _monitor->newPlanningSceneMessage(scene);
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

}


} } }
