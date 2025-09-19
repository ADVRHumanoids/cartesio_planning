#ifndef PLANNING_SCENE_WRAPPER_HXX
#define PLANNING_SCENE_WRAPPER_HXX

#include <mutex>

#include <cartesio_planning/ros2/planning_scene_wrapper.h>

#include <rclcpp/executors.hpp>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>

namespace XBot::Cartesian::Planning {

class PlanningSceneWrapper::Impl
{

public:

    Impl(ModelInterface::ConstPtr model,
         rclcpp::Node::SharedPtr node);

    void update();

    bool checkCollisions() const;

    bool checkSelfCollisions() const;

    double computeCollisionDistance() const;

    double computeSelfCollisionDistance() const;

    std::vector<std::string> getCollidingLinks() const;

    void applyPlanningScene(const moveit_msgs::msg::PlanningScene& scene);

    bool addCollisionObject(moveit_msgs::msg::CollisionObject co,
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


    ~Impl();


private:

    ModelInterface::ConstPtr _model;
    rclcpp::Node::SharedPtr _node;
    rclcpp::Node::SharedPtr _node_mt;

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;

    rclcpp::executors::SingleThreadedExecutor _exe;
    rclcpp::executors::SingleThreadedExecutor _exe_mt;
    std::unique_ptr<std::thread> _th;
    std::atomic_bool _th_exit_flag;

    rclcpp::ServiceBase::SharedPtr _get_ps_srv;
    rclcpp::ServiceBase::SharedPtr _add_octomap_srv;
    rclcpp::ServiceBase::SharedPtr _apply_planning_scene_srv;

    std::mutex _pc_mtx;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> _pc_subs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _point_clouds;

};

}

#endif // PLANNING_SCENE_WRAPPER_HXX
