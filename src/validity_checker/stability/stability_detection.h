#ifndef STABILITY_DETECTION_H
#define STABILITY_DETECTION_H

#include <ros/ros.h>

#include <XBotInterface/ModelInterface.h>
#include <visualization_msgs/MarkerArray.h>
#include <cartesio_planning/SetContactFrames.h>
#include <cartesio_planning/CartesioFrames.h>

class convex_hull;

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

    typedef std::shared_ptr<ConvexHullStability> Ptr;

    ConvexHullStability(XBot::ModelInterface::ConstPtr model);
    ConvexHullStability(XBot::ModelInterface::ConstPtr model, const PolygonFrames& polyframes);


    void setPolygonFrames(const PolygonFrames& pf);
    void addPolygonFrames(const PolygonFrames& pf);
    void removePolygonFrames(const PolygonFrames& pf);

    /**
     * @brief checkStability
     * @return true if stable
     */
    bool checkStability();

    bool getConvexHull(PlanarInclusionDetectionBase::Polygon& poly);

    ~ConvexHullStability();

private:

    std::unique_ptr<convex_hull> _huller;
    PlanarInclusionDetectionBase _inclusion_checker;

    PolygonFrames _polygon_frames;
    XBot::ModelInterface::ConstPtr _model;
};

class ConvexHullROS
{
public:
    typedef std::shared_ptr<ConvexHullROS> Ptr;

    ConvexHullROS(XBot::ModelInterface::ConstPtr model, ConvexHullStability& ch, ros::NodeHandle& nh):
        _ch(ch),
        _model(*model),
        _nh(nh)
    {
        _vis_pub = _nh.advertise<visualization_msgs::Marker>( "convex_hull", 0 );

        _frame_sub = _nh.subscribe("contact_frames", 10, &ConvexHullROS::set_contact_frames, this);

        _frame_srv = _nh.advertiseService("contact_frames", &ConvexHullROS::frame_server, this);
    }

    void publish()
    {
        PlanarInclusionDetectionBase::Polygon poly;
        if(_ch.getConvexHull(poly))
        {
            bool stable = _ch.checkStability();

            visualization_msgs::Marker ch_marker;

            ch_marker.header.frame_id = "world";
            ch_marker.header.stamp = ros::Time::now();
            ch_marker.ns = "convex_hull";
            ch_marker.id = 0;
            ch_marker.type = visualization_msgs::Marker::LINE_STRIP;
            ch_marker.action = visualization_msgs::Marker::ADD;

            Eigen::Vector3d com;
            _model.getCOM(com);

            geometry_msgs::Point p;
            ch_marker.points.clear();
            for(auto pp : poly){
                p.x = pp[0] + com[0];
                p.y = pp[1] + com[1];

                ch_marker.points.push_back(p);
            }

            p.x = poly.at(0)[0] + com[0];
            p.y = poly.at(0)[1] + com[1];
            ch_marker.points.push_back(p);

            if(stable)
            {
                ch_marker.color.a = 1.0;
                ch_marker.color.r = 0.0;
                ch_marker.color.g = 1.0;
                ch_marker.color.b = 0.0;
            }
            else
            {
                ch_marker.color.a = 1.0;
                ch_marker.color.r = 1.0;
                ch_marker.color.g = 0.0;
                ch_marker.color.b = 0.0;
            }

            ch_marker.scale.x = 0.01;

            _vis_pub.publish(ch_marker);
        }
    }

private:

    ConvexHullStability& _ch;
    ros::NodeHandle _nh;
    ros::Publisher _vis_pub;
    const XBot::ModelInterface& _model;
    ros::Subscriber _frame_sub;
    ros::ServiceServer _frame_srv;

    void set_contact_frames(cartesio_planning::SetContactFrames::ConstPtr msg)
    {

        ConvexHullStability::PolygonFrames polyframes;
        for(unsigned int i = 0; i < msg->frames_in_contact.size(); ++i)
            polyframes.push_back(msg->frames_in_contact[i]);


        if(msg->action.data() == msg->SET)
            _ch.setPolygonFrames(polyframes);
        else if(msg->action.data() == msg->ADD)
            _ch.addPolygonFrames(polyframes);
        else if(msg->action.data() == msg->REMOVE)
            _ch.removePolygonFrames(polyframes);
    }

    bool frame_server(cartesio_planning::CartesioFrames::Request& req,
                      cartesio_planning::CartesioFrames::Response& res)
    {
        /* check conditions */

        ConvexHullStability::PolygonFrames polyframes;
        for(unsigned int i = 0; i < req.frames_in_contact.size(); ++i)
            polyframes.push_back(req.frames_in_contact[i]);


        if(req.action.data() == req.SET)
            _ch.setPolygonFrames(polyframes);
        else if(req.action.data() == req.ADD)
            _ch.addPolygonFrames(polyframes);
        else if(req.action.data() == req.REMOVE)
            _ch.removePolygonFrames(polyframes);


        return true;
    }

};

}
}
}


#endif
