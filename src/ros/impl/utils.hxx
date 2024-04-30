#ifndef UTILS_HXX
#define UTILS_HXX

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace utils
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

#endif // UTILS_HXX
