#ifndef __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__
#define __CARTESIO_PLANNING_POSITION_IK_SOLVER_H__

#include <cartesian_interface/CartesianInterfaceImpl.h>

namespace XBot { namespace Cartesian { namespace Planning {

/**
 * @brief The PositionCartesianSolver class implements a position-level IK
 * solver on top of a CartesianInterface object (provided by the user)
 * which solves differential IK. This is done by iterating the local solution
 * and integrating the model state until suitable termination conditions are
 * satisfied.
 */
class PositionCartesianSolver
{

public:

    typedef std::shared_ptr<PositionCartesianSolver> Ptr;

    /**
     * @brief DEFAULT_ERR_TOL is the default tolerance on the error norm for a single constraint.
     */
    static const double DEFAULT_ERR_TOL;

    /**
     * @brief DEFAULT_MAX_ITER is the default maximum number of iterations.
     */
    static const int DEFAULT_MAX_ITER;

    /**
     * @brief PositionCartesianSolver constructor.
     * @param ci
     * @param constrained_ee
     */
    PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci,
                            ProblemDescription ik_problem);

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

    /**
     * @brief getModel
     * @return
     */
    ModelInterface::Ptr getModel() const;

    /**
     * @brief getErrorThreshold
     * @return
     */
    double getErrorThreshold() const;

private:

    struct TaskData
    {
        typedef std::shared_ptr<TaskData> Ptr;

        TaskData(int size);

        const int size;
        Eigen::MatrixXd J;
        Eigen::VectorXd error;

        virtual void update(CartesianInterfaceImpl::Ptr ci,
                            ModelInterface::Ptr model) = 0;

        virtual ~TaskData();
    };

    struct CartesianTaskData : TaskData
    {
        CartesianTaskData(std::string distal_link,
                          std::string base_link,
                          std::vector<int> indices);

        std::string distal_link;
        std::string base_link;
        std::vector<int> indices;

        void update(CartesianInterfaceImpl::Ptr ci,
                    ModelInterface::Ptr model) override;
    };

    int _n_task;
    CartesianInterfaceImpl::Ptr _ci;
    ModelInterface::Ptr _model;
    std::map<std::string, TaskData::Ptr> _task_map;
    int _max_iter;
    double _err_tol;

    std::function<void(void)> _iter_callback;

};


} } }


#endif
