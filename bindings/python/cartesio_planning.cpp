#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "planner/cartesio_ompl_planner.h"
#include "goal/goal_sampler.h"

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

auto goal_sample_construct = [](CartesianInterfaceImpl::Ptr ci)
{
    return GoalSamplerBase(std::make_shared<PositionCartesianSolver>(ci));
};

auto goal_sample_setref = [](GoalSamplerBase& self,
                            const std::string& frame,
                            const Eigen::Affine3d& Tref)
{
    self.getIkSolver()->setDesiredPose(frame, Tref);
};

struct TimedOut : public std::runtime_error
{
    using runtime_error::runtime_error;
};

auto goal_sample = [](GoalSamplerBase& self,
                      double timeout_sec)
{
    Eigen::VectorXd q;
    if(self.sampleGoal(q, timeout_sec))
    {
        return q;
    }

    throw TimedOut("Goal sampler timed out");
};

PYBIND11_MODULE(planning, m)
{

    py::register_exception<TimedOut>(m, "TimedOut");

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

    py::class_<GoalSamplerBase>(m, "GoalSampler")
            .def(py::init(goal_sample_construct))
            .def("sampleGoal", goal_sample)
            .def("setValidityChecker", &GoalSamplerBase::setValidityCheker)
            .def("setDesiredPose", goal_sample_setref);
}

