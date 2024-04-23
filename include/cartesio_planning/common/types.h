#ifndef CARTESIO_PLANNING_TYPES_H
#define CARTESIO_PLANNING_TYPES_H

#include <Eigen/Dense>

#define CARTESIO_PLANNING_DECLARE_SMART_PTR(Class) \
typedef std::shared_ptr<Class> Ptr; \
    typedef std::shared_ptr<const Class> ConstPtr; \
    typedef std::weak_ptr<Class> WeakPtr; \
    typedef std::unique_ptr<Class> UniquePtr;

namespace XBot::Cartesian::Planning {



}



#endif // TYPES_H
