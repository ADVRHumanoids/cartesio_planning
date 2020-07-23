#ifndef CENTROIDAL_STATICS_H
#define CENTROIDAL_STATICS_H

#include <OpenSoT/utils/ForceOptimization.h>
#include <ros/ros.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <cartesio_planning/SetContactFrames.h>
#include <eigen_conversions/eigen_msg.h>
#include <yaml-cpp/yaml.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesio_acceleration_support/DynamicFeasibility.h>
#include <cartesio_acceleration_support/FrictionCone.h>
#include <cartesio_acceleration_support/Force.h>



namespace XBot {
namespace Cartesian {
namespace Planning {

class CentroidalStatics{

public:
    /**
     * @brief CentroidalStatics checks if the robot is statically stable with given contacts.
     * The problem which is optimized depends if the contact moments are optimized or not.
     *
     * If optimize_torque param is set to false (default), contact points are considered and the optimized problem is:
     *
     * 1.     min || statics ||
     *          st
     *              friction_cones
     *              wrench_limits <-- here moments are set to 0!
     *
     * 2.     min || contact_wrench ||
     *          st
     *              (1.)
     *
     * While if optimize_torque is set to true, surface contacts are considered, therefore:
     *
     * 1.     min || statics ||
     *          st
     *              friction_cones
     *              center_of_pressure
     *              wrench_limits
     *
     * 2.     min || contact_wrench ||
     *          st
     *              (1.)
     *
     * notice that we add the center_of_pressure of each contact which has to remain inside the contact support polygon.
     *
     * @param model of the robot
     * @param contact_links vector of links in contact
     * @param friction_coeff of the contacts (we consider a single friction coefficient for all the contacts in the constructor)
     * @param optimize_torque if contact moments are optimized (default false)
     * @param xlims_cop x limits [xl, xu] (intended in the local frame) for the center of pressure
     * if contact moments are optimized ([0,0]) default (we consider a single limit for all the contacts in the constructor)
     * @param ylims_cop y limits [yl, yu] (intended in the local frame) for the center of pressure
     * if contact moments are optimized ([0,0]) default (we consider a single limit for all the contacts in the constructor)
     */
    CentroidalStatics(XBot::ModelInterface::ConstPtr model, const std::vector<std::string>& contact_links,
                      const double friction_coeff, const bool optimize_torque = false,
                      const Eigen::Vector2d& xlims_cop = Eigen::Vector2d::Zero(2),
                      const Eigen::Vector2d& ylims_cop = Eigen::Vector2d::Zero(2));

    /**
     * @brief setOptimizeTorque permits to enable or disable the optimization of contact moments
     * @param optimize_torque
     */
    void setOptimizeTorque(const bool optimize_torque);

    /**
     * @brief setFrictionCoeff set friction coefficient for all the contacts
     * @param friction_coeff
     * @return false if friction_coeff < 0.
     */
    bool setFrictionCoeff(const double friction_coeff);

    /**
     * @brief setContactLinks permits to set a new vector of contact links (rotations are initialized as identity)
     * @param contact_links
     */
    void setContactLinks(const std::vector<std::string>& contact_links);

    /**
     * @brief addContactLinks add contact links to existing once (rotations are initialized as identity)
     * @param contact_links
     */
    void addContactLinks(const std::vector<std::string>& contact_links);

    /**
     * @brief removeContactLinks remove contact links from existing once
     * @param contact_links
     */
    void removeContactLinks(const std::vector<std::string>& contact_links);

    /**
     * @brief setContactRotationMatrix permits to change contact rotation matrix for a particular friciton cone constraint
     * associated to a contact link
     * @param contact_link
     * @param w_R_c
     * @return false if no friction cones are associated to that contact link
     */
    bool setContactRotationMatrix(const std::string& contact_link,
                                  const Eigen::Matrix3d& w_R_c);

    /**
     * @brief getFrictionCones
     * @return map of <link_names, friction cones>
     */
    const std::map<std::string, XBot::Cartesian::acceleration::FrictionCone::Ptr>& getFrictionCones(){ return _fcs;}

    /**
     * @brief getFricitonCoefficient
     * @return friction coefficient
     */
    double getFricitonCoefficient(){ return _friction_coeff;}


    /**
     * @brief checkStability
     * @param init_solver set to false to do not initialize internal solver (first time is initialize automatically)
     * @return true if stable
     */
    bool checkStability(const double eps = 1e-3);

    /**
     * @brief getForces
     * @return map of link name and associated contact forces
     */
    const std::map<std::string, Eigen::Vector6d>& getForces();


private:
    YAML::Node createYAMLProblem(const std::vector<std::string>& contact_links,
                           const double friction_coeff,
                           const bool optimize_torque);

    void init();

    bool compute();

    XBot::ModelInterface::ConstPtr _model;
    XBot::ModelInterface::Ptr _model_internal;

    std::map<std::string, XBot::Cartesian::acceleration::FrictionCone::Ptr> _fcs;
    std::map<std::string, XBot::Cartesian::acceleration::ForceTask::Ptr> _fs;
    bool _optimize_torque;
    double _friction_coeff;

    XBot::Cartesian::CartesianInterface::Ptr _ci;
    XBot::Cartesian::acceleration::DynamicFeasibility::Ptr _dyn_feas;

    std::vector<std::string> _contact_links;

    Eigen::Vector2d _x_lims_cop;
    Eigen::Vector2d _y_lims_cop;

    std::map<std::string, Eigen::Vector6d> _Fc;

};

///**
// * @brief The CentroidalStaticsROS class
// * NOTE: contact position will change only when contacts are changed/updated
// */
class CentroidalStaticsROS
{
public:
    typedef std::shared_ptr<CentroidalStaticsROS> Ptr;

    CentroidalStaticsROS(XBot::ModelInterface::ConstPtr model, CentroidalStatics& cs, ros::NodeHandle& nh):
        _cs(cs),
        _model(*model),
        _nh(nh),
        _tf_prefix(""),
        _eps(1e-3)
    {
        _contact_sub = _nh.subscribe("contacts", 10, &CentroidalStaticsROS::set_contacts, this);

        _vis_pub = _nh.advertise<visualization_msgs::Marker>("centroidal_statics/forces", 0);

        std::string tmp;
        if(_nh.getParam("tf_prefix", tmp))
            _tf_prefix = tmp;
        double eps;
        if(nh.getParam("eps", eps))
            _eps = eps;
    }


    void publish()
    {
        _fcs = _cs.getFrictionCones();

        if(_fcs.size() > 0)
        {
            bool check_stability =  _cs.checkStability(_eps);

            std::map<std::string, Eigen::Vector6d> Fcs = _cs.getForces();

            int i = 0; int k = 0;
            ros::Time t = ros::Time::now();
            for(auto fc : _fcs)
            {
                Eigen::Vector6d F = Fcs[fc.first];

                //std::cout<<contact.first<<"  "<<F.transpose()<<std::endl;

                Eigen::Affine3d w_T_c;
                _model.getPose(fc.first, w_T_c);
                Eigen::Matrix6d Adj; Adj.setIdentity();
                Adj.block(0,0,3,3) = w_T_c.linear().transpose();
                Adj.block(3,3,3,3) = w_T_c.linear().transpose();
                F = Adj*F;

                //Piselloni (Forze)
                visualization_msgs::Marker marker;
                marker.header.frame_id = _tf_prefix+fc.first;
                marker.header.stamp = t;
                marker.ns = "computed_contact_forces";
                marker.id = i;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                geometry_msgs::Point p;
                p.x = 0.; p.y = 0.; p.z = 0.;
                marker.points.push_back(p);

                double Fg = _model.getMass()*9.81;
                p.x = F[0]/Fg;
                p.y = F[1]/Fg;
                p.z = F[2]/Fg;

                marker.points.push_back(p);

                marker.scale.x = 0.03;
                marker.scale.y = 0.09;
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

                //Piselloni (Momenti)
                marker.ns = "computed_contact_moments";
                marker.id = i;
                marker.type = visualization_msgs::Marker::ARROW;
                marker.action = visualization_msgs::Marker::ADD;

                marker.points.clear();
                p.x = 0.; p.y = 0.; p.z = 0.;
                marker.points.push_back(p);

                double alpha = 10.;
                p.x = alpha*F[3]/Fg;
                p.y = alpha*F[4]/Fg;
                p.z = alpha*F[5]/Fg;

                marker.points.push_back(p);

                marker.scale.x = 0.03;
                marker.scale.y = 0.09;
                marker.scale.z = 0.001;
                marker.color.a = 1.0; // Don't forget to set the alpha!


                if(check_stability)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 0.0;
                    marker.color.b = 1.0;
                }
                else
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }

                _vis_pub.publish( marker );
                i++;

                //Friction Cones
                marker.points.clear();
                marker.ns = "friction_cone";
                marker.id = i;
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
                marker.action = visualization_msgs::Marker::ADD;

                Eigen::Matrix3d w_R_fc = fc.second->getContactFrame();
                Eigen::Matrix3d w_R_c = w_T_c.linear();
                Eigen::Matrix3d c_R_fc = w_R_c.transpose()*w_R_fc;
                Eigen::Matrix3d RotY; RotY.setIdentity();
                RotY(0,0) = std::cos(-M_PI_2); RotY(0,2) = std::sin(-M_PI_2);
                RotY(2,0) = -std::sin(-M_PI_2); RotY(2,2) = std::cos(-M_PI_2);
                c_R_fc = c_R_fc*RotY;
                Eigen::Quaterniond q(c_R_fc);
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();
                marker.pose.orientation.w = q.w();


                geometry_msgs::Point pp[3];
                static const double DELTA_THETA = M_PI/16.0;
                double theta = 0.;
                double scale = 0.09;
                double angle = M_PI_2-std::atan(_cs.getFricitonCoefficient());
                for (std::size_t i = 0; i < 32; i++)
                {
                   pp[0].x = 0;
                   pp[0].y = 0;
                   pp[0].z = 0;

                   pp[1].x = scale;
                   pp[1].y = scale * cos(theta) / angle;
                   pp[1].z = scale * sin(theta) / angle;

                   pp[2].x = scale;
                   pp[2].y = scale * cos(theta + DELTA_THETA) / angle;
                   pp[2].z = scale * sin(theta + DELTA_THETA) / angle;

                   marker.points.push_back(pp[0]);
                   marker.points.push_back(pp[1]);
                   marker.points.push_back(pp[2]);

                   theta += DELTA_THETA;
                 }

                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;

                if(check_stability)
                {
                    marker.color.r = 0.0;
                    marker.color.g = 0.5;
                    marker.color.b = 0.5;
                }
                else
                {
                    marker.color.r = 1.0;
                    marker.color.g = 0.0;
                    marker.color.b = 0.0;
                }


                _vis_pub.publish( marker );


                i++;

                k++;

            }



        }




    }

private:
    /**
     * @brief set_contacts
     * NOTE: when SET and ADD are used, the friction coefficient is updated with the one of the message which is the same for
     * all the contacts
     * @param msg
     */
public: void set_contacts(cartesio_planning::SetContactFrames::ConstPtr msg)
    {
        if(msg->action.data() == msg->SET)
        {
            _cs.setContactLinks(msg->frames_in_contact);
            _cs.setFrictionCoeff(msg->friction_coefficient);
        }
        else if(msg->action.data() == msg->ADD)
        {
            _cs.addContactLinks(msg->frames_in_contact);
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

        _cs.setOptimizeTorque(msg->optimize_torque);

    }

private:
    CentroidalStatics& _cs;
    const XBot::ModelInterface& _model;
    ros::NodeHandle _nh;
    ros::Subscriber _contact_sub;

    std::map<std::string, XBot::Cartesian::acceleration::FrictionCone::Ptr> _fcs;

    ros::Publisher _vis_pub;

    std::string _tf_prefix;

    double _eps;



};

}
}
}

#endif
