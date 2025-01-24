#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

#include <XBotInterface/ModelInterface.h>

#include <std_srvs/Trigger.h>

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
     * @brief start server for octomap compatibility
     */
    bool startOctomapServer();

    /**
     * @brief update octomap from subscribed point clouds
     */
    bool updateOctomap();

    /**
     * @brief clearOctomap
     */
    void clearOctomap();

    /**
     * @brief updateOctomap
     * @param pc_topic
     * @param local_min
     * @param local_max
     * @param base_min
     * @param base_max
     * @return
     */
    bool updateOctomapFromTopic(std::string pc_topic,
                                double resolution,
                                double ground_height,
                                Eigen::Vector3d local_min, Eigen::Vector3d local_max,
                                Eigen::Vector3d base_min, Eigen::Vector3d base_max, 
                                std::string base_link="base_link");

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
    bool checkCollisions(bool verbose=false) const;   
    bool checkSelfCollisions(bool verbose=false) const;

    double computeCollisionDistance() const;
    double computeSelfCollisionDistance() const;

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
     * @brief set padding around the robot links for more conservative
     * collision detection
     */
    void setLinkPadding(std::map<std::string, double> padding);

    /**
     * @brief applyPlanningScene
     * @param scene
     */
    bool applyPlanningScene(const moveit_msgs::PlanningScene& scene);

    /**
     * @brief addCollisionObject
     * @param co
     * @param attach_to_link
     * @param touch_links
     * @return
     */
    bool addCollisionObject(moveit_msgs::CollisionObject co,
                            std::string attach_to_link = "",
                            std::vector<std::string> touch_links = std::vector<std::string>{});

    /**
     * @brief addBox
     * @param id
     * @param size
     * @param T
     * @param frame_id
     * @param attach_to_link
     * @param touch_links
     * @return
     */

    bool addBox(std::string id,
                const Eigen::Vector3d& size,
                const Eigen::Affine3d& T,
                std::string frame_id = "world",
                std::string attach_to_link = "",
                std::vector<std::string> touch_links = std::vector<std::string>{});

    /**
     * @brief addSphere
     * @param id
     * @param radius
     * @param T
     * @param frame_id
     * @param attach_to_link
     * @param touch_links
     * @return
     */
    bool addSphere(std::string id,
                   double radius,
                   const Eigen::Affine3d& T,
                   std::string frame_id = "world",
                   std::string attach_to_link = "",
                   std::vector<std::string> touch_links = std::vector<std::string>{});

    /**
     * @brief addCylinder
     * @param id
     * @param radius
     * @param height
     * @param T
     * @param frame_id
     * @param attach_to_link
     * @param touch_links
     * @return
     */
    bool addCylinder(std::string id,
                     double radius,
                     double  height,
                     const Eigen::Affine3d& T,
                     std::string frame_id = "world",
                     std::string attach_to_link = "",
                     std::vector<std::string> touch_links = std::vector<std::string>{});

    /**
     * @brief getPlanningScene
     * @param req
     * @param res
     * @return
     */
    bool getPlanningScene(moveit_msgs::GetPlanningScene::Request& req,
                          moveit_msgs::GetPlanningScene::Response& res);

    /**
     * @brief clearPlanningScene
     */
    void clearPlanningScene();
    
    mutable collision_detection::AllowedCollisionMatrix acm;

private:

    void computeChainToLinks();
    bool octomap_service(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
    bool apply_planning_scene_service(moveit_msgs::ApplyPlanningScene::Request & req, moveit_msgs::ApplyPlanningScene::Response & res);
    
    void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg, const int& i, const std::string& base_link);

    void transform_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, std::string frame_id);

    std::map<std::string, std::set<std::string>> _chain_to_links;
    std::map<std::string, std::string> _link_to_chain;

    tf::TransformListener _listener;

    ModelInterface::ConstPtr _model;

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;

    ros::CallbackQueue _queue;
    ros::AsyncSpinner _async_spinner;
    ros::ServiceServer _get_ps_srv;
    ros::ServiceServer _add_octomap_srv;
    ros::ServiceServer _apply_planning_scene_srv;
    
    std::mutex _pc_mtx;
    std::vector<ros::Subscriber> _pc_subs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _point_clouds;

    srdf_advr::Model _srdf;

    double _octomap_resolution;
    std::string _octomap_base_link;
    bool filterOutPlanningSceneObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
    void computeCollisionObjectVertices(const moveit_msgs::CollisionObject& obj, std::vector<Eigen::Vector3d>& vertices);
    void computeAttachedCollisionObjectVertices(const moveit_msgs::AttachedCollisionObject& obj, std::vector<Eigen::Vector3d>& vertices);
    void filterWithConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, const std::vector<Eigen::Vector3d>& vertices);
    bool filterOutClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);
    bool _filter_out_planning_scene_objects;
    double _filter_out_objects_pad;
    double _filter_out_attached_objects_pad;
    std::vector<std::string> _ignored_planning_scene_objects;
    moveit_msgs::GetPlanningScene _get_planning_scene_srv;
    pcl::CropHull<pcl::PointXYZ> _crop_hull_filter;
    pcl::ExtractIndices<pcl::PointXYZ> _extract_indeces_filter;
    Eigen::Affine3d w_T_obj, b_T_w, b_T_obj, bAttached_T_aObj, b_T_aObj, w_T_bAttached;
    bool _filter_out_clusters;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr _cluster_tree;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> _cluster_extractor;

};

} } }

#endif // COLLISION_DETECTION_H
