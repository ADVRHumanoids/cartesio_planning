#include <cartesio_planning/cartesio_planning.h>
#include <cartesio_planning/state_validity_checker/collision.h>
#include <cartesio_planning/constraints/cartesian_constraint.h>
#include <cartesio_planning/constraints/contact_constraint.h>
#include <cartesio_planning/trajectory_interpolation.h>

#include <cartesio_planning/ros2/planning_scene_wrapper.h>
#include <cartesio_planning/ros2/robot_viz.h>
#include <cartesio_planning/ros2/robot_viz_dummy_checker.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace XBot::Cartesian::Planning;
namespace py = pybind11;
using rvp = py::return_value_policy;


// constraint trampoline
class PyConstraint : public Constraint
{
public:

    CARTESIO_DECLARE_SMART_PTR(PyConstraint);

    using Constraint::Constraint;

    Eigen::VectorXd _value(const Eigen::VectorXd& q) const override
    {
        PYBIND11_OVERLOAD_PURE(
            Eigen::VectorXd,
            Constraint,
            _value,
            q
        );
    }

    Eigen::MatrixXd _jacobian(const Eigen::VectorXd& q) const override
    {
        PYBIND11_OVERLOAD_PURE(
            Eigen::MatrixXd,
            Constraint,
            _jacobian,
            q
        );
    }

    int _constraintSize() const override
    {
        PYBIND11_OVERLOAD_PURE(
            int,
            Constraint,
            _constraintSize
        );
    }

};


void eigenToStd(Eigen::Ref<const Eigen::VectorXd> qeig,
                std::vector<double>& qvec)
{
    qvec.resize(qeig.size());

    Eigen::VectorXd::Map(qvec.data(), qvec.size()) = qeig;
}

Planner::Ptr make_planner(StateSpace::ConstPtr ss,
                          std::string opts)
{
    return std::make_shared<Planner>(ss, YAML::Load(opts));
}

int ss_add_robot_configuration_space(StateSpace &self,
                                     XBot::ModelInterface::Ptr model)
{
    return self.addRobotConfigurationSpace(model);
}

std::pair<bool, std::vector<std::string>> ss_check_valid(StateSpace& self,
                                                         const Eigen::VectorXd& q)
{
    std::vector<std::string> failed_checks;

    bool valid = self.checkValid(q, &failed_checks);

    return {valid, failed_checks};
}

bool svc_check_valid(StateValidityChecker& self,
                     const Eigen::VectorXd &q)
{
    std::optional<Eigen::VectorXd> qnear;
    return self.checkValid(q, qnear);
}

PyConstraint::Ptr make_pyconstraint(std::shared_ptr<const StateSpace> space,
                                           std::string options)
{
    return std::make_shared<PyConstraint>(space, YAML::Load(options));
}


CartesianConstraint::Ptr make_cartesian_constraint(std::shared_ptr<const StateSpace> space,
                                                   XBot::Cartesian::CartesianInterfaceImpl::Ptr ci,
                                                   std::string options)
{
    return std::make_shared<CartesianConstraint>(space, ci, YAML::Load(options));
}

ContactConstraint::Ptr make_contact_constraint(std::shared_ptr<const StateSpace> space,
                                               XBot::ModelInterface::Ptr model,
                                               std::map<std::string, std::vector<int>> contact_map,
                                               std::string options)
{
    return std::make_shared<ContactConstraint>(space, model, contact_map, YAML::Load(options));
}

PYBIND11_MODULE(pycartesio_planning, m)
{
    py::class_<Constraint, PyConstraint, Constraint::Ptr>(m, "Constraint")
        .def(py::init<std::shared_ptr<const StateSpace>, std::string>())
        .def("constraintSize", &Constraint::constraintSize)
        .def("bind", &Constraint::bind)
        .def("checkJacobian", &Constraint::checkJacobian)
        .def("jacobian", &Constraint::jacobian)
        .def("setRefineTarget", &Constraint::setRefineTarget)
        .def("project", [](Constraint& self, Eigen::VectorXd q)
             {
                self.project(q);
                return q;
             }
        )
        .def("refine", [](Constraint& self, Eigen::VectorXd q)
             {
                 self.refine(q);
                 return q;
             }
             )
        .def("sample", &Constraint::sample)
        .def("sampleGaussian", &Constraint::sampleGaussian)
        .def("value", &Constraint::value)
        ;

    py::class_<CartesianConstraint, Constraint, CartesianConstraint::Ptr>(m, "CartesianConstraint")
        .def(py::init(&make_cartesian_constraint),
             py::arg("state_space"),
             py::arg("ci"),
             py::arg("yaml_options") = "")
        ;

    py::class_<ContactConstraint, Constraint, ContactConstraint::Ptr>(m, "ContactConstraint")
        .def(py::init(&make_contact_constraint),
             py::arg("state_space"),
             py::arg("model"),
             py::arg("contact_map"),
             py::arg("yaml_options") = "")
        .def("resetContactPose", &ContactConstraint::resetContactPose)
        .def("setContactPose", &ContactConstraint::setContactPose)
        ;

    py::class_<StateValidityChecker, StateValidityChecker::Ptr>(m, "StateValidityChecker")
        .def("checkValid", svc_check_valid)
        .def("getInvalidStateInformation",
             [](const StateValidityChecker& self)
             {
                 std::ostringstream oss;
                 self.printInvalidStateInformation(oss);
                 return oss.str();
             })
        ;

    py::class_<CollisionValidityChecker, StateValidityChecker, CollisionValidityChecker::Ptr>(
        m, "CollisionValidityChecker")
        .def(py::init<StateSpace::ConstPtr,
                      XBot::Collision::CollisionModel::Ptr,
                      std::optional<std::string>,
                      int>(),
             py::arg("state_space"), py::arg("collision_model"), py::arg("id") = "collision",
             py::arg("substate_idx") = 0)
        .def("setIncludeEnvironment", &CollisionValidityChecker::setIncludeEnvironment)
        .def("setThreshold", &CollisionValidityChecker::setThreshold)
        .def("setCallback", &CollisionValidityChecker::setCallback)
        ;

    py::class_<StateSpace, StateSpace::Ptr>(m, "StateSpace")
        .def(py::init())
        .def("addRobotConfigurationSpace", ss_add_robot_configuration_space,
             py::arg("model"))
        .def("ambientRandom", &StateSpace::ambientRandom)
        .def("random", &StateSpace::random)
        .def("interpolate", &StateSpace::interpolate)
        .def("setConstraint", &StateSpace::setConstraint)
        .def("addStateValidityChecker", &StateSpace::addStateValidityChecker)
        .def("checkValid", ss_check_valid)
        ;

    py::class_<Planner, Planner::Ptr>(m, "Planner")
        .def(py::init(&make_planner),
             py::arg("state_space"), py::arg("options") = "")
        .def("solve", &Planner::solve,
             py::arg("qstart"), py::arg("qgoal"), py::arg("timeout"), py::arg("planner_type"))
        .def("getSolutionPath", &Planner::getSolutionPath,
             py::arg("simplify") = false, py::arg("timeout") = -1)
        ;

    m.def("simpleTrajectoryInterpolation",
          [](StateSpace& ss, Eigen::MatrixXd wp, double max_vel, double max_acc, double dt)
          {
              auto trj = simpleInterpolation(ss,
                                             wp,
                                             Eigen::VectorXd::Constant(ss.getNv(), max_vel),
                                             Eigen::VectorXd::Constant(ss.getNv(), max_acc),
                                             dt);

              Eigen::VectorXd time(trj.points.size());
              Eigen::MatrixXd pos(ss.getNq(), time.size());
              Eigen::MatrixXd vel(ss.getNv(), time.size()),
                  acc(ss.getNv(), time.size());

              for(int i = 0; i < time.size(); i++)
              {
                  time[i] = trj.points[i].time_from_start;
                  pos.col(i) = Eigen::VectorXd::Map(trj.points[i].positions.data(), ss.getNq());
                  vel.col(i) = Eigen::VectorXd::Map(trj.points[i].velocities.data(), ss.getNv());
                  acc.col(i) = Eigen::VectorXd::Map(trj.points[i].accelerations.data(), ss.getNv());

              }

              return std::make_tuple(time, pos, vel, acc);
          },
        py::arg("state_space"), py::arg("waypoints"), py::arg("max_vel"), py::arg("max_acc"), py::arg("dt"));


    // ROS
    auto mros = m.def_submodule("ros");

    mros.def("init_rclcpp",
             [](std::string name,
                std::vector<std::string> args)
             {
                 std::vector<const char*> aargs;
                 for(auto& a : args) aargs.push_back(a.c_str());
                 int argc = args.size();
                 rclcpp::init(argc, (char**)aargs.data(), rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
             },
        py::arg("name"), py::arg("args") = std::vector<std::string>());

    py::class_<PlanningSceneWrapper, PlanningSceneWrapper::Ptr>(mros, "PlanningSceneWrapper")
        .def(py::init<XBot::ModelInterface::ConstPtr>(),
             py::arg("model"))
        .def("update", &PlanningSceneWrapper::update)
        .def("addBox", &PlanningSceneWrapper::addBox,
             py::arg("id"),
             py::arg("size"),
             py::arg("T"),
             py::arg("frame_id") = "world",
             py::arg("attach_to_link") = "",
             py::arg("touch_links") = std::vector<std::string>())
        ;

    py::class_<PlanningSceneChecker, StateValidityChecker, PlanningSceneChecker::Ptr>(mros, "PlanningSceneChecker")
        .def(py::init<PlanningSceneWrapper::Ptr,
                      StateSpace::ConstPtr,
                      std::string,
                      int>(),
             py::arg("psw"),
             py::arg("state_space"),
             py::arg("name") = "planning_scene",
             py::arg("substate_idx") = 0)
        ;


    py::class_<RobotViz, RobotViz::Ptr>(mros, "RobotViz")
        .def(py::init<XBot::ModelInterface::ConstPtr,
                      std::string,
                      std::optional<RobotViz::color>>(),
             py::arg("model"),
             py::arg("topic"),
             py::arg("color") = Eigen::Vector4d(1, 0, 0, 1))
        .def("publishMarkers",
             [](RobotViz& self, std::vector<std::string> red_links)
             {
                self.publishMarkers(self.getNode().get_clock()->now(), red_links);
            }, py::arg("red_links") = std::vector<std::string>())
        ;
}
