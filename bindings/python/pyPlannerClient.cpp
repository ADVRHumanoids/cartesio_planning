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
            .def("callPlanner", &PlannerClient::callPlanner,
                 py::arg("time"),
                 py::arg("planner_type"),
                 py::arg("interpolation_time"),
                 py::arg("trajectory_space") = "",
                 py::arg("distal_links") = std::vector<std::string>(),
                 py::arg("base_links") = std::vector<std::string>())
            .def("setContactFrames", &PlannerClient::setContactFrames)
            .def("getJointTrajectory", &PlannerClient::getJointTrajectory)
            .def("getCartesianTrajectory", &PlannerClient::getCartesianTrajectory)
        ;
}

