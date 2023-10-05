#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>

#include "goal/NSPG.h"
#include "goal/goal_sampler.h"

namespace py = pybind11;
using namespace XBot::Cartesian::Planning;

auto goal_sampler_basic_sample(GoalSamplerBase& self, double timeout)
{
    Eigen::VectorXd q;
    return self.sampleGoal(q, timeout);
}

PYBIND11_MODULE(NSPG, m)
{
    py::class_<NSPG, NSPG::Ptr>(m, "NSPG")
        .def(py::init<PositionCartesianSolver::Ptr, 
                      ValidityCheckContext>(),
             py::arg("ik_solver"),
             py::arg("vc_context"))
        .def("sample", &NSPG::sample)
        .def("getIKSolver", &NSPG::getIKSolver)
        .def("getModel", &NSPG::getModel);

    py::class_<GoalSamplerBase, GoalSamplerBase::Ptr>(m, "GoalSamplerBasic")
        .def(py::init<PositionCartesianSolver::Ptr,
                      ValidityCheckContext>(),
             py::arg("ik_solver"),
             py::arg("vc_context"))
        .def("sample", goal_sampler_basic_sample)
        .def("generateRandomSeed", &GoalSamplerBase::generateRandomSeed)
        .def("setJointLimits", &GoalSamplerBase::setJointLimits)
        .def("setIterationCallback", &GoalSamplerBase::setIterationCallback)
        ;
}

