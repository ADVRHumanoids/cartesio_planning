#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <XBotInterface/ModelInterface.h>


namespace XBot { namespace Cartesian { namespace Planning {

/**
 * @brief The CollisionDetection class provides collision detection capabilities
 * to the ModelInterface class. Changes to the environment are monitored internally
 * from suitable ROS topics between calls to startMonitor() and stopMonitor()
 */
class PlanningSceneWrapper
{

public:

    typedef std::shared_ptr<PlanningSceneWrapper> Ptr;

    /**
     * @brief CollisionDetection constructor. The class keeps a model pointer,
     * which is used for querying the robot state at each call to update()
     * @param model
     */
    PlanningSceneWrapper(ModelInterface::ConstPtr model);

    /**
     * @brief startMonitor method starts monitoring changes in the environments
     * from topics, in a separate thread.
     */
    void startMonitor();

    /**
     * @brief stopMonitor method stops monitoring changes in the environment
     */
    void stopMonitor();

    /**
     * @brief startGetPlanningSceneServer
     */
    void startGetPlanningSceneServer();

    /**
     * @brief update method updates the internal collision detector model state
     * from the provided pointer to ModelInterface
     */
    void update();

    /**
     * @brief checkCollisions method checks the ModelInterface state at last call to
     * update() for collisions, either between robot links or with the environment.
     * @return true if collisions were found
     */
    bool checkCollisions() const;   
    bool checkSelfCollisions() const;

    /**
     * @brief Get the names of the links that are involved in collisions for the current state
     */
    std::vector<std::string> getCollidingLinks() const;

    /**
     * @brief getCollidingChains
     * @return vector of colliding chains (computed from colliding links)
     */
    std::vector<XBot::ModelChain> getCollidingChains() const;

    /**
     * @brief set padding around the robot links for more conservative
     * collision detection
     */
    void setPadding(double padding);

    /**
     * @brief applyPlanningScene
     * @param scene
     */
    void applyPlanningScene(const moveit_msgs::PlanningScene& scene);

    /**
     * @brief getPlanningScene
     * @param req
     * @param res
     * @return
     */
    bool getPlanningScene(moveit_msgs::GetPlanningScene::Request& req,
                          moveit_msgs::GetPlanningScene::Response& res);
    
    mutable collision_detection::AllowedCollisionMatrix acm;

private:

    void computeChainToLinks();

    std::map<std::string, std::set<std::string>> _chain_to_links;
    std::map<std::string, std::string> _link_to_chain;

    ModelInterface::ConstPtr _model;

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;

    ros::CallbackQueue _queue;
    ros::AsyncSpinner _async_spinner;
    ros::ServiceServer _get_ps_srv;

    srdf_advr::Model _srdf;

};

} } }

#endif // COLLISION_DETECTION_H