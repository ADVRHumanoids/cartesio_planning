#include "position_ik_solver.h"

using namespace XBot::Cartesian::Planning;

const double PositionCartesianSolver::DEFAULT_ERR_TOL = 1e-4;
const int PositionCartesianSolver::DEFAULT_MAX_ITER = 100;

PositionCartesianSolver::PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci,
                                                 std::vector<std::string> planning_ee):
    _n_task(0),
    _ci(ci),
    _model(ci->getModel()),
    _max_iter(DEFAULT_MAX_ITER),
    _err_tol(DEFAULT_ERR_TOL),
    _iter_callback([](){})
{
    for(auto ee : planning_ee)
    {
        Task t;
        _ci->getCurrentPose(ee, t.des_pose);
        _task_map[ee] = t;
        _n_task += 6;
    }
}

void PositionCartesianSolver::setDesiredPose(std::string distal_frame,
                                             const Eigen::Affine3d & pose)
{
    if(!_ci->setPoseReference(distal_frame, pose))
    {
        throw std::runtime_error("Unable to set desired pose for task '" + distal_frame + "'");
    }
}



bool PositionCartesianSolver::solve()
{
    // allocate variables
    Eigen::VectorXd q, dq, error;

    // main solver loop
    bool tol_satisfied = false;
    int iter = 0;
    while(!tol_satisfied && iter < _max_iter)
    {

        double dt = 0.1; // dummy dt
        if(!_ci->update(0.0, dt))
        {
            return false;
        }

        _model->getJointPosition(q);
        _model->getJointVelocity(dq);
        q += dq*dt;
        _model->setJointPosition(q);
        _model->update();

        getError(error);
        tol_satisfied = error.norm() < _err_tol*_n_task;
//        tol_satisfied = tol_satisfied && dq.norm() < 1e-3;

        printf("Error at iter #%d is %f \n", iter, error.norm());
        iter++;

        _iter_callback();

    }

    return tol_satisfied;

}

void PositionCartesianSolver::getError(Eigen::VectorXd& error) const
{
    error.setZero(_n_task);

    int error_idx = 0;
    for(auto pair : _task_map)
    {

        std::string distal_frame = pair.first;
        std::string base_frame = _ci->getBaseLink(distal_frame);

        Eigen::Affine3d T, Tdes;
        _ci->getCurrentPose(distal_frame, T);
        _ci->getPoseReference(distal_frame, Tdes);

        Eigen::Vector3d pos_error = T.translation() - Tdes.translation();

        Eigen::Vector3d rot_error;
        XBot::Utils::computeOrientationError(Tdes.linear(),
                                             T.linear(),
                                             rot_error);

        error.segment<6>(error_idx) << pos_error, rot_error;
        error_idx += 6;

    }

}

void PositionCartesianSolver::getJacobian(Eigen::MatrixXd & J) const
{
    J.setZero(_n_task, _model->getJointNum());

    int jac_idx = 0;
    for(auto pair : _task_map)
    {

        std::string distal_frame = pair.first;
        std::string base_frame = _ci->getBaseLink(distal_frame);

        Eigen::MatrixXd Ji;

        if(base_frame == "world")
        {
            _model->getJacobian(distal_frame, Ji);
        }
        else
        {
            _model->getJacobian(distal_frame, base_frame, Ji);
        }

        J.middleRows<6>(jac_idx) = Ji;
        jac_idx += 6;

    }
}

int PositionCartesianSolver::getSize() const
{
    return _n_task;
}

void PositionCartesianSolver::setIterCallback(std::function<void ()> f)
{
    _iter_callback = f;
}
