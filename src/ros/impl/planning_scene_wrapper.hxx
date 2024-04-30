#ifndef PLANNING_SCENE_WRAPPER_HXX
#define PLANNING_SCENE_WRAPPER_HXX

#include <cartesio_planning/ros/planning_scene_wrapper.h>

#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace XBot::Cartesian::Planning {

class PlanningSceneWrapper::Impl
{

public:

    Impl(ModelInterface::ConstPtr model);

    void update();

    bool checkCollisions() const;

    bool checkSelfCollisions() const;

    double computeCollisionDistance() const;

    double computeSelfCollisionDistance() const;

    std::vector<std::string> getCollidingLinks() const;

    void applyPlanningScene(const moveit_msgs::PlanningScene& scene);

    bool addCollisionObject(moveit_msgs::CollisionObject co,
                            std::string attach_to_link,
                            std::vector<std::string> touch_links = std::vector<std::string>{});

    bool addBox(std::string id,
                const Eigen::Vector3d& size,
                const Eigen::Affine3d& T,
                std::string frame_id ,
                std::string attach_to_link,
                std::vector<std::string> touch_links = std::vector<std::string>{});


    bool addSphere(std::string id,
                   double radius,
                   const Eigen::Affine3d& T,
                   std::string frame_id,
                   std::string attach_to_link,
                   std::vector<std::string> touch_links = std::vector<std::string>{});

    bool addCylinder(std::string id,
                     double radius,
                     double  height,
                     const Eigen::Affine3d& T,
                     std::string frame_id,
                     std::string attach_to_link,
                     std::vector<std::string> touch_links = std::vector<std::string>{});


private:

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

};

}

#endif // PLANNING_SCENE_WRAPPER_HXX
