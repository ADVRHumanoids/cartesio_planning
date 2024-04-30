#ifndef PLANNING_SCENE_WRAPPER_H
#define PLANNING_SCENE_WRAPPER_H

#include "../common/types.h"

#include "../state_validity_checker.h"

#include <xbot2_interface/xbotinterface2.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace XBot::Cartesian::Planning {

class PlanningSceneWrapper
{

public:

    CARTESIO_PLANNING_DECLARE_SMART_PTR(PlanningSceneWrapper)

    /**
     * @brief PlanningSceneWrapper
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
     * @brief start server for octomap compatibility
     */
    void startOctomapServer(std::vector<std::string> input_topics);

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
                                Eigen::Vector3d base_min, Eigen::Vector3d base_max);

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
    void applyPlanningScene(const moveit_msgs::PlanningScene& scene);

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


    ~PlanningSceneWrapper();

private:

    class Impl;

    std::unique_ptr<Impl> impl;

};

class PlanningSceneChecker : public StateValidityChecker
{

public:

    PlanningSceneChecker(PlanningSceneWrapper::Ptr ps,
                         StateSpace::ConstPtr space,
                         std::string name = "planning_scene",
                         int substate_idx = 0);

    bool checkValid(const Eigen::VectorXd &q, std::optional<Eigen::VectorXd> &qnear) const override;

    void printInvalidStateInformation(std::ostream &os) const override;

private:

    PlanningSceneWrapper::Ptr _ps;

};

}

#endif // PLANNING_SCENE_WRAPPER_H
