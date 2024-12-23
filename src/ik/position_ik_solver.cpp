#include <cartesio_planning/ik/position_ik_solver.h>

using namespace XBot;
using namespace XBot::Cartesian::Planning;

const double PositionCartesianSolver::DEFAULT_ERR_TOL = 1e-4;
const int PositionCartesianSolver::DEFAULT_MAX_ITER = 100;//60;

PositionCartesianSolver::PositionCartesianSolver(CartesianInterfaceImpl::Ptr ci):
    _n_task(0),
    _ci(ci),
    _model(ci->getModel()),
    _max_iter(DEFAULT_MAX_ITER),
    _err_tol(DEFAULT_ERR_TOL),
    _iter_callback([](){}),
    _min_step_size(1e-3)
{
    for(auto t : ci->getIkProblem().getTask(0))
    {
        if(auto cart = std::dynamic_pointer_cast<Cartesian::CartesianTask>(t))
        {
            auto tdata = std::make_shared<CartesianTaskData>(cart);

            _n_task += tdata->size;

            _task_map[cart->getName()] = tdata;
            _ctask_map[cart->getName()] = tdata;

            printf("[PositionCartesianSolver] adding cartesian task '%s' to '%s', size is %d \n",
                   cart->getBaseLink().c_str(),
                   cart->getDistalLink().c_str(),
                   tdata->size);
        }
    }
}

void PositionCartesianSolver::setDesiredPose(std::string distal_frame,
                                             const Eigen::Affine3d & pose)
{
    if (_ctask_map.find(distal_frame) == _ctask_map.end()) {
        throw std::runtime_error("Task named '" + distal_frame + "' not found!");
    }
    
    bool ok = _ctask_map.at(distal_frame)->ctask->setPoseReference(pose);

    if(!ok)
    {
        throw std::runtime_error("Unable to set desired pose for task '" + distal_frame + "'");
    }
}

void PositionCartesianSolver::setMinStepSize(double min_step_size)
{
    _min_step_size = min_step_size;

    if (_min_step_size > 0.5)
        ROS_INFO("Line search is disabled");
}

Eigen::Affine3d PositionCartesianSolver::getDesiredPose(std::string distal_frame) const
{
    Eigen::Affine3d T;
    _ctask_map.at(distal_frame)->ctask->getPoseReferenceRaw(T);
    return T;
}


bool PositionCartesianSolver::solve()
{
    // allocate variables
    Eigen::VectorXd q, qcurr, dq, error;

    // initial q
    _model->getJointPosition(qcurr);

    // initial cost
    getError(error);
    double current_cost = error.squaredNorm();
    double achieved_cost = current_cost;

    // main solver loop
    bool tol_satisfied = false;
//    bool tol_satisfied = error.lpNorm<Eigen::Infinity>() < _err_tol;
    int iter = 0;
    while(!tol_satisfied && iter < _max_iter)
    {
        double dt = 1.0; // dummy dt

        // compute search direction dq
        if(!_ci->update(0.0, dt))
        {
            return false;
        }

        _model->getJointVelocity(dq);

        // line search
        double step_size = 1.0;

        // todo: armijo rule
        while(achieved_cost >= current_cost)
        {
//            printf("cost %4.3e >= %4.3e, try alpha = %f \n",
//                   achieved_cost, current_cost, step_size);

            if(step_size < _min_step_size)
            {
//                 fprintf(stderr, "warn: step too small (error = %f > %f) \n",
//                         error.lpNorm<Eigen::Infinity>(), _err_tol);
                break;
            }

            q = qcurr + step_size*dq;
            _model->setJointPosition(q);
            _model->update();
            getError(error);
            achieved_cost = error.squaredNorm();


            step_size *= 0.5;
        }

//        printf("cost %4.3e < %4.3e, ok \n",
//               achieved_cost, current_cost);

        current_cost = achieved_cost;
        qcurr = q;

        if(_ros_server)
        {
            _ros_server->run();
        }

        tol_satisfied = error.lpNorm<Eigen::Infinity>() < _err_tol;

        iter++;

        _iter_callback();

    }

    if(!tol_satisfied)
    {
        fprintf(stderr, "ik failed: max iteration reached (error = %f > %f) \n",
                error.lpNorm<Eigen::Infinity>(), _err_tol);
    }

    return tol_satisfied;

}

void PositionCartesianSolver::getError(Eigen::VectorXd& error) const
{
    error.setZero(_n_task);

    int error_idx = 0;
    for(auto pair : _task_map)
    {
        auto t = pair.second;
        t->update(*_model); // tbd: optimize


        error.segment(error_idx, t->size) = t->error;
        error_idx += t->size;
    }

}

void PositionCartesianSolver::getJacobian(Eigen::MatrixXd & J) const
{
    J.setZero(_n_task, _model->getJointNum());

    int jac_idx = 0;
    for(auto pair : _task_map)
    {
        auto t = pair.second;
        t->update(*_model); // tbd: optimize

        J.middleRows(jac_idx, t->size) = t->J;
        jac_idx += t->size;
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

XBot::ModelInterface::Ptr PositionCartesianSolver::getModel() const
{
    return _model;
}

double PositionCartesianSolver::getErrorThreshold() const
{
    return _err_tol * _n_task;
}

void PositionCartesianSolver::setRosServerClass(XBot::Cartesian::RosServerClass::Ptr ros_server)
{
    _ros_server = ros_server;
}

void PositionCartesianSolver::reset()
{
    _ci->reset(0.0);
}

void PositionCartesianSolver::setErrorTolerance(const double error_tolerance)
{
    _err_tol = error_tolerance;
}

void PositionCartesianSolver::setMaxIterations(const int max_iter)
{
    _max_iter = max_iter;
}

PositionCartesianSolver::TaskData::TaskData(Cartesian::TaskDescription::Ptr t):
    size(t->getIndices().size()),
    task(t)
{
    error.setZero(size);
    J.setZero(size, 0);
}

PositionCartesianSolver::TaskData::~TaskData()
{

}

PositionCartesianSolver::CartesianTaskData::CartesianTaskData(CartesianTask::Ptr t):
    TaskData(t),
    distal_link(t->getDistalLink()),
    base_link(t->getBaseLink()),
    ctask(t)
{

}

void PositionCartesianSolver::CartesianTaskData::update(XBot::ModelInterface& model)
{
    J.setZero(size, model.getJointNum());
    error.setZero(size);

    /* If task was disabled, error and jacobian are zero */
    auto active = task->getActivationState();

    if(active == ActivationState::Disabled)
    {
        return;
    }


    /* Error computation */
    Eigen::Affine3d T, Tdes;

    //see issue https://github.com/ADVRHumanoids/cartesio_planning/issues/21
    //ctask->getCurrentPose(T);
    model.getPose(ctask->getDistalLink(), ctask->getBaseLink(), T);

    ctask->getPoseReference(Tdes);

    Eigen::Vector3d pos_error = T.translation() - Tdes.translation();
    Eigen::Vector3d rot_error;
    Eigen::Matrix3d L;
    compute_orientation_error(Tdes.linear(),
                              T.linear(),
                              rot_error,
                              L);

    /* Body jacobian */
    if(ctask->isSubtaskLocal())
    {
        pos_error = T.linear().transpose()*pos_error;
        rot_error = T.linear().transpose()*rot_error;
    }

    Eigen::Vector6d error6d;
    error6d << pos_error, rot_error;

    const auto& indices = task->getIndices();

    for(int i = 0; i < size; i++)
    {
        error[i] = error6d[indices[i]];
    }


    /* Jacobian computation */
    Eigen::MatrixXd Ji;

    if(base_link == "world")
    {
        model.getJacobian(distal_link, Ji);
    }
    else
    {
        model.getJacobian(distal_link, base_link, Ji);
    }

    Ji.bottomRows<3>() = L * Ji.bottomRows<3>();

    /* Body jacobian */
    if(ctask->isSubtaskLocal())
    {
        Ji.topRows<3>() = T.linear().transpose()*Ji.topRows<3>();
        Ji.bottomRows<3>() = T.linear().transpose()*Ji.bottomRows<3>();
    }

    for(int i = 0; i < size; i++)
    {
        J.row(i) = Ji.row(indices[i]);
    }

}

void PositionCartesianSolver::CartesianTaskData::compute_orientation_error(const Eigen::Matrix3d & Rd,
                                                                           const Eigen::Matrix3d & Re,
                                                                           Eigen::Vector3d & e_o,
                                                                           Eigen::Matrix3d & L)
{
    auto S = XBot::Utils::skewSymmetricMatrix;

    e_o = 0.5 * (Re.col(0).cross(Rd.col(0)) + Re.col(1).cross(Rd.col(1)) + Re.col(2).cross(Rd.col(2)));
    L = 0.5 * ( S(Rd.col(0))*S(Re.col(0)) + S(Rd.col(1))*S(Re.col(1)) + S(Rd.col(2))*S(Re.col(2)));
}

Cartesian::CartesianInterfaceImpl::Ptr PositionCartesianSolver::getCI()
{
    return _ci;
}

