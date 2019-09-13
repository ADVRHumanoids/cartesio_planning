#ifndef __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__
#define __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__

#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace XBot { namespace Cartesian { namespace Planning {

/**
 * @brief The PositionCartesianSolver class
 */
class PositionCartesianSolver
{

public:

    static const double DEFAULT_ERR_TOL;

    static const int DEFAULT_MAX_ITER;

    /**
     * @brief PositionCartesianSolver
     * @param ci
     * @param constrained_ee
     */
    PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci,
                            std::vector<std::string> constrained_ee);

    /**
     * @brief setDesiredPose
     * @param distal_frame
     * @param pose
     */
    void setDesiredPose(std::string distal_frame,
                        const Eigen::Affine3d& pose);

    /**
     * @brief solve
     * @return
     */
    bool solve();

    /**
     * @brief getError
     * @return
     */
    void getError(Eigen::VectorXd& error) const;

    /**
     * @brief getJacobian
     * @param J
     */
    void getJacobian(Eigen::MatrixXd& J) const;

    /**
     * @brief getSize
     * @return
     */
    int getSize() const;

    /**
     * @brief setIterCallback
     * @param f
     */
    void setIterCallback(std::function<void(void)> f);

private:

    struct Task
    {
    };

    int _n_task;
    CartesianInterfaceImpl::Ptr _ci;
    ModelInterface::Ptr _model;
    std::map<std::string, Task> _task_map;
    int _max_iter;
    double _err_tol;

    std::function<void(void)> _iter_callback;

};


} } }


#endif
