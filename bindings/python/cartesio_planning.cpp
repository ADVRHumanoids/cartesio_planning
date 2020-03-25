#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "planner/cartesio_ompl_planner.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Planning;

auto ompl_planner_construct = [](Eigen::VectorXd qmin, Eigen::VectorXd qmax, std::string yaml_str)
{
    return OmplPlanner(qmin, qmax, YAML::Load(yaml_str));
};

auto ompl_planner_print = [](OmplPlanner& self)
{
    self.print();
};

PYBIND11_MODULE(planning, m)
{

    py::class_<OmplPlanner>(m, "OmplPlanner")
            .def(py::init(ompl_planner_construct), py::arg("qmin"), py::arg("qmax"), py::arg("yaml")="")
            .def("__repr__", ompl_planner_print)
            .def("setStateValidityPredicate", &OmplPlanner::setStateValidityPredicate)
            .def("setStartAndGoalStates",
                 static_cast<void(OmplPlanner::*)(const Eigen::VectorXd&,
                                                  const Eigen::VectorXd&,
                                                  const double)>(&OmplPlanner::setStartAndGoalStates))
            .def("getSolutionPath", &OmplPlanner::getSolutionPath)
            .def("solve", &OmplPlanner::solve);
}

