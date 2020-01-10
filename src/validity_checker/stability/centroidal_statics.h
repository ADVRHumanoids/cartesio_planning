#ifndef CENTROIDAL_STATICS_H
#define CENTROIDAL_STATICS_H

#include <OpenSoT/utils/ForceOptimization.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <cartesio_planning/SetContactFrames.h>
#include <eigen_conversions/eigen_msg.h>


namespace XBot {
namespace Cartesian {
namespace Planning {

class CentroidalStatics{

public:
    /**
     * @brief CentroidalStatics
     * @param model
     * @param contact_links
     * @param friction_coeff
     * @param optimize_torque if contact moments are optimized
     */
    CentroidalStatics(XBot::ModelInterface::ConstPtr model, const std::vector<std::string>& contact_links,
                      const double friction_coeff, const bool optimize_torque);

    bool setFrictionCoeff(const double friction_coeff);
    void setOptimizeTorque(const bool optimize_torque);

    void setContactLinks(const std::vector<std::string>& contact_links);
    void addContatcLinks(const std::vector<std::string>& contact_links);
    void removeContactLinks(const std::vector<std::string>& contact_links);

    bool setContactRotationMatrix(const std::string& contact_link,
                                  const Eigen::Matrix3d& w_R_c);

    const std::map<std::string, Eigen::Matrix3d>& getContacts(){ return _contacts;}
    double getFricitonCoefficient(){ return _friction_coeff;}


    /**
     * @brief checkStability
     * @param init_solver set to false to do not initialize internal solver (first time is initialize automatically)
     * @return true if stable
     */
    bool checkStability(const double eps = 1e-6, const bool init_solver = true);

    const Eigen::VectorXd& getTorques(){ return _tau;}
    const std::vector<Eigen::Vector6d>& getForces(){ return _Fc;}

private:
    /**
     * @brief init initialize contact force optimization
     */
    void init();

    /**
     * @brief compute contact forces and tau joints
     * @param Fc
     * @param tau
     * @return true if solver success
     */
    bool compute();

    OpenSoT::utils::ForceOptimization::Ptr _force_optim;
    XBot::ModelInterface::ConstPtr _model;

    std::map<std::string, Eigen::Matrix3d> _contacts;
    bool _optimize_torque;
    double _friction_coeff;

    bool _is_initialized;

    std::vector<Eigen::Vector6d> _Fc;
    Eigen::VectorXd _tau;

};

/**
 * @brief The CentroidalStaticsROS class
 * NOTE: contact position will change only when contacts are changed/updated
 */
class CentroidalStaticsROS
{
public:
    typedef std::shared_ptr<CentroidalStaticsROS> Ptr;

    CentroidalStaticsROS(XBot::ModelInterface::ConstPtr model, CentroidalStatics& cs, ros::NodeHandle& nh):
        _cs(cs),
        _model(*model),
        _nh(nh)
    {
        _visual_tools = std::make_shared<rviz_visual_tools::RvizVisualTools>("ci/world", "centroidal_statics/friciton_cones");

        _visual_tools->setAlpha(0.3);
        _visual_tools->enableBatchPublishing(true);
        _visual_tools->loadMarkerPub(true, true);

        _contact_sub = _nh.subscribe("contacts", 10, &CentroidalStaticsROS::set_contacts, this);

        _vis_pub = _nh.advertise<visualization_msgs::Marker>("centroidal_statics/forces", 0);
    }

    void publish()
    {
        double mu = _cs.getFricitonCoefficient();

        std::map<std::string, Eigen::Matrix3d> contacts = _cs.getContacts();

        if(!map_compare(_contacts, contacts))
        {
            _contacts = contacts;

            _visual_tools->deleteAllMarkers();

            for(auto const& contact : _contacts)
            {
                //1) We get the pose of the link in contact world frame
                Eigen::Affine3d w_T_c;
                _model.getPose(contact.first, w_T_c);

                //2) we substitute the roation with the one stored in the contact
                w_T_c.linear() = contact.second;

                //3) cones are published around the X axis when Identity is used, we need to locally rotate about -90 on the Y axis
                Eigen::Matrix3d RotY; RotY.setIdentity();
                RotY(0,0) = std::cos(-M_PI_2); RotY(0,2) = std::sin(-M_PI_2);
                RotY(2,0) = -std::sin(-M_PI_2); RotY(2,2) = std::cos(-M_PI_2);
                w_T_c.linear() = w_T_c.linear()*RotY;

                _visual_tools->publishCone(w_T_c, M_PI_2-std::atan(mu), rviz_visual_tools::GREEN, 0.07);
            }
        }

        if(_contacts.size() > 0)
        {
            //we should check that contacts did not move
            bool check_stability =  _cs.checkStability(1e-6, false);

            std::vector<Eigen::Vector6d> Fcs = _cs.getForces();

            int i = 0;
            ros::Time t = ros::Time::now();
            for(auto const& contact : _contacts)
            {
                Eigen::Vector6d F = Fcs[i];

                visualization_msgs::Marker marker;
                marker.header.frame_id = "ci/"+contact.first;
                marker.header.stamp = t;
                marker.ns = "computed_contact_forces";
                marker.id = i;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                geometry_msgs::Point p;
                p.x = 0.; p.y = 0.; p.z = 0.;
                marker.points.push_back(p);
                p.x = F[0]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                p.y = F[1]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                p.z = F[2]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                marker.points.push_back(p);

                marker.scale.x = 0.05;
                marker.scale.y = 0.1;
                marker.color.a = 1.0; // Don't forget to set the alpha!

                if(check_stability)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 1.0;
                    marker.color.b = 0.0;
                }
                else
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }

                _vis_pub.publish( marker );

                i++;

            }



        }


        _visual_tools->trigger();


    }

private:
    template <typename Map>
    bool map_compare (Map const &lhs, Map const &rhs) {
        // No predicate needed because there is operator== for pairs already.
        return lhs.size() == rhs.size()
            && std::equal(lhs.begin(), lhs.end(),
                          rhs.begin());
    }

    /**
     * @brief set_contacts
     * NOTE: when SET and ADD are used, the friction coefficient is updated with the one of the message which is the same for
     * all the contacts
     * @param msg
     */
    void set_contacts(cartesio_planning::SetContactFrames::ConstPtr msg)
    {
        if(msg->action.data() == msg->SET)
        {
            _cs.setContactLinks(msg->frames_in_contact);
            _cs.setFrictionCoeff(msg->friction_coefficient);
        }
        else if(msg->action.data() == msg->ADD)
        {
            _cs.addContatcLinks(msg->frames_in_contact);
            _cs.setFrictionCoeff(msg->friction_coefficient);
        }
        else if(msg->action.data() == msg->REMOVE)
            _cs.removeContactLinks(msg->frames_in_contact);


        if(!msg->rotations.empty() && (msg->action.data() == msg->ADD || msg->action.data() == msg->SET))
        {
            if(msg->rotations.size() != msg->frames_in_contact.size())
                ROS_ERROR("msg->rotations.size() != msg->frames_in_contact.size(), rotations will not be applied!");
            else
            {
                for(unsigned int i = 0; i < msg->rotations.size(); ++i)
                {
                    Eigen::Quaterniond q;
                    tf::quaternionMsgToEigen(msg->rotations[i], q);

                    _cs.setContactRotationMatrix(msg->frames_in_contact[i], q.toRotationMatrix());
                }
            }
        }

    }

    CentroidalStatics& _cs;
    const XBot::ModelInterface& _model;
    ros::NodeHandle _nh;
    ros::Subscriber _contact_sub;

    std::map<std::string, Eigen::Matrix3d> _contacts;

    rviz_visual_tools::RvizVisualToolsPtr _visual_tools;
    ros::Publisher _vis_pub;
};

}
}
}

#endif
