#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <cartesio_planning/planner_client.h>


namespace py = pybind11;;

PYBIND11_MODULE(pyplan, m) {

    py::class_<PlannerClient>(m, "PlannerClient")
            .def(py::init())
            .def("setStartState", &PlannerClient::setStartState)
            .def("setGoalState", &PlannerClient::setGoalState)
            .def("callPlanner", &PlannerClient::callPlanner)
            .def("setContactFrames", &PlannerClient::setContactFrames)
        ;
}

