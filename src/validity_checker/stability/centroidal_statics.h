#ifndef CENTROIDAL_STATICS_H
#define CENTROIDAL_STATICS_H

#include <OpenSoT/utils/ForceOptimization.h>

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

}
}
}

#endif
