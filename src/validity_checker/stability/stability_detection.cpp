#include "validity_checker/stability/stability_detection.h"

#include "cartesian_utils.h"
#include "convex_hull_utils.h"

using namespace XBot::Cartesian::Planning;



ConvexHullStability::ConvexHullStability(XBot::ModelInterface::ConstPtr model):
    _model(model),
    _huller(new convex_hull)
{

}

ConvexHullStability::ConvexHullStability(XBot::ModelInterface::ConstPtr model, const PolygonFrames& polyframes):
    _model(model),
    _huller(new convex_hull)
{
    setPolygonFrames(polyframes);
}

bool ConvexHullStability::getConvexHull(PlanarInclusionDetectionBase::Polygon& poly)
{
    if(_polygon_frames.empty())
            return false;

    std::list<KDL::Vector> points_KDL;
    if(!_huller->getSupportPolygonPoints(points_KDL, _polygon_frames, *_model, "COM"))
        return false;

    std::vector<KDL::Vector> ch_KDL;
    if(!_huller->getConvexHull(points_KDL, ch_KDL))
        return false;

    PlanarInclusionDetectionBase::Point2 p;
    for(unsigned int i = 0; i < ch_KDL.size(); ++i)
    {
        p[0] = ch_KDL[i].x();
        p[1] = ch_KDL[i].y();
        poly.push_back(p);
    }

    return true;
}


bool ConvexHullStability::checkStability()
{
    PlanarInclusionDetectionBase::Polygon poly;
    if(!getConvexHull(poly))
        return false;

    Eigen::Vector3d com;
    com.setZero(); //since the convex hull is omputed wrt com, we check against (0,0)

    _inclusion_checker.setPolygon(poly);
    return _inclusion_checker.pointInPolygon(com.head(2));
}

void ConvexHullStability::setPolygonFrames(const PolygonFrames &pf)
{
    _polygon_frames = pf;
}

void ConvexHullStability::addPolygonFrames(const PolygonFrames& pf)
{
    for(auto frame : pf)
        _polygon_frames.push_back(frame);
}

void ConvexHullStability::removePolygonFrames(const PolygonFrames& pf)
{
    for(auto frame : pf)
    {
        auto it = std::find(_polygon_frames.begin(), _polygon_frames.end(), frame);
        if(it != _polygon_frames.end())
            _polygon_frames.erase(it);
    }
}



PlanarInclusionDetectionBase::PlanarInclusionDetectionBase()
{

}

PlanarInclusionDetectionBase::PlanarInclusionDetectionBase(const Polygon &polygon)
{
    setPolygon(polygon);
}

void PlanarInclusionDetectionBase::setPolygon(const Polygon &polygon)
{
    _polygon_vertices = polygon;
}

const PlanarInclusionDetectionBase::Polygon& PlanarInclusionDetectionBase::getPolygon() const
{
    return _polygon_vertices;
}

bool PlanarInclusionDetectionBase::pointInPolygon(const Point2 &point) const
{
    std::vector<float> X;X.reserve(_polygon_vertices.size());
    std::vector<float> Y;Y.reserve(_polygon_vertices.size());
    for(auto p : _polygon_vertices)
    {
        X.push_back(p[0]);
        Y.push_back(p[1]);
    }
    if(cartesian_utils::pnpoly(_polygon_vertices.size(), X.data(), Y.data(), (float)point[0], (float)point[1]) == 0)
        return false;
    return true;
}

ConvexHullStability::~ConvexHullStability()
{

}
