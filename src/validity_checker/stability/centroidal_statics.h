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

    /**
     * @brief init initialize contact force optimization
     */
    void init();

private:
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
        _cs.init();

        _contact_sub = _nh.subscribe("contacts", 10, &CentroidalStaticsROS::set_contacts, this);

        _vis_pub = _nh.advertise<visualization_msgs::Marker>("centroidal_statics/forces", 0);
    }


    void publish()
    {
        _contacts = _cs.getContacts();

        if(_contacts.size() > 0)
        {
            bool check_stability =  _cs.checkStability(1e-6, false);

            std::vector<Eigen::Vector6d> Fcs = _cs.getForces();

            int i = 0; int k = 0;
            ros::Time t = ros::Time::now();
            for(auto const& contact : _contacts)
            {
                Eigen::Vector6d F = Fcs[k];

                //std::cout<<contact.first<<"  "<<F.transpose()<<std::endl;

                Eigen::Affine3d w_T_c;
                _model.getPose(contact.first, w_T_c);
                Eigen::Matrix6d Adj; Adj.setIdentity();
                Adj.block(0,0,3,3) = w_T_c.linear().transpose();
                Adj.block(3,3,3,3) = w_T_c.linear().transpose();
                F = Adj*F;

                //Piselloni
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
                if(F.segment(0,3).norm() < 1e-3)
                    p.x = p.y = p.z = 0.0;
                else
                {
                    p.x = F[0]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                    p.y = F[1]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                    p.z = F[2]/std::sqrt(std::pow(F[0],2) + std::pow(F[1],2) + std::pow(F[2],2));
                }
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

                //Friction Cones
                marker.points.clear();
                marker.ns = "friction_cone";
                marker.id = i;
                marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
                marker.action = visualization_msgs::Marker::ADD;

                Eigen::Matrix3d w_R_fc = contact.second;
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

private:
    CentroidalStatics& _cs;
    const XBot::ModelInterface& _model;
    ros::NodeHandle _nh;
    ros::Subscriber _contact_sub;

    std::map<std::string, Eigen::Matrix3d> _contacts;

    ros::Publisher _vis_pub;


};

}
}
}

#endif
