#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <XBotInterface/ModelInterface.h>


namespace XBot { namespace Cartesian { namespace Planning {


class CollisionDetection
{

public:

    CollisionDetection(ModelInterface::ConstPtr model);

    void startMonitor();

    void stopMonitor();

    void update();

    bool checkCollisions() const;

private:

    ModelInterface::ConstPtr _model;

    planning_scene_monitor::PlanningSceneMonitorPtr _monitor;

};

} } }

#endif // COLLISION_DETECTION_H
