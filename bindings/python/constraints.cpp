#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "constraints/cartesian_constraint.h"

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Planning;

auto cc_construct = [](CartesianInterfaceImpl::Ptr ci)
{
    return std::make_shared<CartesianConstraint>(std::make_shared<PositionCartesianSolver>(ci));
};

struct ProjectionFailure : std::runtime_error
{
    using runtime_error::runtime_error;
};

auto cc_project = [](ompl::base::Constraint& self, const Eigen::VectorXd& q)
{
    Eigen::VectorXd qproj = q;
    if(!self.project(qproj))
    {
        throw ProjectionFailure("project() failed");
    }

    return qproj;
};

auto cc_fun = [](ompl::base::Constraint& self, const Eigen::VectorXd& q)
{
    Eigen::VectorXd f(self.getCoDimension());

    self.function(q, f);

    return f;
};

auto cc_jacob = [](ompl::base::Constraint& self, const Eigen::VectorXd& q)
{
    Eigen::MatrixXd J(self.getCoDimension(), self.getAmbientDimension());

    self.jacobian(q, J);

    return J;
};

PYBIND11_MODULE(constraints, m)
{
    py::register_exception<ProjectionFailure>(m, "ProjectionFailure");

    py::class_<ompl::base::Constraint,
            ompl::base::ConstraintPtr>(m, "Constraint")
            .def("project", cc_project)
            .def("function", cc_fun)
            .def("jacobian", cc_jacob);

    py::class_<CartesianConstraint, ompl::base::Constraint,
            CartesianConstraint::Ptr>(m, "CartesianConstraint")
            .def(py::init(cc_construct))
            .def("reset", &CartesianConstraint::reset);
}
