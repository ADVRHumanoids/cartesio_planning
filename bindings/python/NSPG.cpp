#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "goal/NSPG.h"

namespace py = pybind11;
using namespace XBot::Cartesian::Planning;

PYBIND11_MODULE(NSPG, m)
{
    py::class_<NSPG>(m, "NSPG")
        .def(py::init<PositionCartesianSolver::Ptr, 
                      ValidityCheckContext>(),
             py::arg("ik_solver"),
             py::arg("vc_context"))
        .def("sample", &NSPG::sample);             
}
