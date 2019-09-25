#ifndef SELF_COLLISION_CHECKER_H
#define SELF_COLLISION_CHECKER_H

#include <XBotInterface/ModelInterface.h>
#include <kdl/frames.hpp>
#include <fcl/config.h>
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/distance.h"


#if FCL_MINOR_VERSION <= 3
    template <typename T>
    using shared_ptr =  boost::shared_ptr<T>;
#else
    template <typename T>
    using shared_ptr =  std::shared_ptr<T>;
#endif


namespace XBot { namespace Cartesian { namespace Planning {

class SelfCollisionChecker
{
public:
    typedef std::shared_ptr<SelfCollisionChecker> Ptr;

    struct collision_info
    {
        boost::shared_ptr<fcl::CollisionObject<double>> collision_object;
        shared_ptr<fcl::CollisionGeometry<double>> shape;
        /**
         * @brief link_T_shape a transforms from link frame to shape frame.
         *        Notice how the shape frame is always the center of the shape,
         *        except for the capsule, where it lies on one endpoint
         */
        KDL::Frame link_T_shape;
    };

public: // Constraint interface

    SelfCollisionChecker(XBot::ModelInterface& model);

    /**
     * @brief updateCollisionObjects updates all collision objects with correct transforms (link_T_shape)
     */
    void updateCollisionObjects();

    bool inCollision(const std::string& link1, const std::string& link2);
private:
    XBot::ModelInterface& _model;

    std::map<std::string, collision_info> _collision_infos;

    /**
     * @brief _links_to_update a list of links to update
     */
    std::set<std::string> _links_to_update;

    inline KDL::Vector toKdl(urdf::Vector3 v)
    {
      return KDL::Vector(v.x, v.y, v.z);
    }

    inline KDL::Rotation toKdl(urdf::Rotation r)
    {
      return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
    }

    inline KDL::Frame toKdl(urdf::Pose p)
    {
      return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
    }

    inline fcl::Transform3<double> tofcl ( const KDL::Frame &in )
    {
        fcl::Transform3<double> out;
        double x,y,z,w;
        in.M.GetQuaternion ( x, y, z, w );
        fcl::Quaternion<double> q ( w, x, y, z );
        out.translation() << in.p[0], in.p[1], in.p[2];
        out.linear() = q.toRotationMatrix();
        return out;
    }




};

} } }

#endif // SELF_COLLISION_CHECKER_H
