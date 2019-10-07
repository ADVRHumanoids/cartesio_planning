#ifndef STABILITY_DETECTION_H
#define STABILITY_DETECTION_H

#include <OpenSoT/utils/cartesian_utils.h>
#include <OpenSoT/utils/convex_hull_utils.h>
#include <XBotInterface/ModelInterface.h>

namespace XBot { namespace Cartesian { namespace Planning {

class PlanarInclusionDetectionBase
{
public:
    typedef Eigen::Vector2d Point2;
    typedef std::vector<Point2> Polygon;

    PlanarInclusionDetectionBase();
    PlanarInclusionDetectionBase(const Polygon& polygon);

    bool pointInPolygon(const Point2& point) const;
    void setPolygon(const Polygon& polygon);
    const Polygon& getPolygon() const;
protected:
    Polygon _polygon_vertices;

};

class ConvexHullStability
{
public:
    typedef std::list<std::string> PolygonFrames;

    ConvexHullStability(XBot::ModelInterface::ConstPtr model);
    ConvexHullStability(XBot::ModelInterface::ConstPtr model, const PolygonFrames& polyframes);


    void setPolygonFrames(const PolygonFrames& pf);
    /**
     * @brief checkStability
     * @return true if stable
     */
    bool checkStability();

    bool getConvexHull(PlanarInclusionDetectionBase::Polygon& poly);
private:
    convex_hull _huller;
    PlanarInclusionDetectionBase _inclusion_checker;

    PolygonFrames _polygon_frames;
    XBot::ModelInterface::Ptr _model;
};

}
}
}


#endif
