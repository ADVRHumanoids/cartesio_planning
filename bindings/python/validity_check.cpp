#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include <cartesio_planning/planner/cartesio_ompl_planner.h>
#include <cartesio_planning/validity_checker/planning_scene_wrapper.h>
#include "validity_checker/stability/centroidal_statics.h"
#include <cartesio_planning/validity_checker/validity_checker_context.h>

#include <ros/serialization.h>
#include <eigen_conversions/eigen_msg.h>

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Planning;

auto remove_collision_object = [](PlanningSceneWrapper& self,
                                  std::string id)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.operation = co.REMOVE;

    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;
    ps.world.collision_objects.push_back(co);

    self.applyPlanningScene(ps);
};

auto create_validity_check_context = [](std::string vc_node, ModelInterface::Ptr model)
{
    ros::NodeHandle nh;
    ValidityCheckContext vc_context(YAML::Load(vc_node), model, nh);
    
    return vc_context;
};

auto set_contact_links = [](CentroidalStatics& self, const std::vector<std::string>& active_links, bool log = false)
{
    self.setContactLinks(active_links);
    self.init(log);
};

auto set_contact_links_and_optimize_torque = [](CentroidalStatics& self,
                                                const std::vector<std::string>& active_links, bool optimize_torque, bool log = false)
{
    self.setContactLinks(active_links);
    self.setOptimizeTorque(optimize_torque);
    self.init(log);
};

auto set_optimize_torque = [](CentroidalStatics& self, bool optimize_torque, bool log = false)
{
    self.setOptimizeTorque(optimize_torque);
    self.init(log);
};

auto add_checker = [](ValidityCheckContext& self, std::string id, std::function<bool()> fn)
{
    return self.vc_aggregate.add(fn, id);
};

auto rm_checker = [](ValidityCheckContext& self, std::string id)
{
    return self.vc_aggregate.remove(id);
};


PYBIND11_MODULE(validity_check, m)
{

    py::class_<PlanningSceneWrapper, PlanningSceneWrapper::Ptr>(m, "PlanningSceneWrapper")
        .def(py::init<ModelInterface::ConstPtr>())
        .def("startMonitor", &PlanningSceneWrapper::startMonitor)
        .def("stopMonitor", &PlanningSceneWrapper::stopMonitor)
        .def("startGetPlanningSceneServer", &PlanningSceneWrapper::startGetPlanningSceneServer)
        .def("startOctomapServer", &PlanningSceneWrapper::startOctomapServer)
        .def("updateOctomap", &PlanningSceneWrapper::updateOctomap)
        .def("clearOctomap", &PlanningSceneWrapper::clearOctomap)
        .def("updateOctomapFromTopic", &PlanningSceneWrapper::updateOctomapFromTopic,
             py::arg("topic"), py::arg("resolution"), py::arg("ground_height"),
             py::arg("local_lb") = Eigen::Vector3d::Zero(),
             py::arg("local_ub") = Eigen::Vector3d::Zero(),
             py::arg("base_lb") = Eigen::Vector3d::Zero(),
             py::arg("base_ub") = Eigen::Vector3d::Zero(),
             py::arg("base_link") = "base_link")
        .def("setPadding", &PlanningSceneWrapper::setPadding)
        .def("setLinkPadding", &PlanningSceneWrapper::setLinkPadding)
        .def("checkCollisions", &PlanningSceneWrapper::checkCollisions)
        .def("checkSelfCollisions", &PlanningSceneWrapper::checkSelfCollisions)
        .def("computeSelfCollisionDistance", &PlanningSceneWrapper::computeSelfCollisionDistance)
        .def("computeCollisionDistance", &PlanningSceneWrapper::computeCollisionDistance)
        .def("getCollidingLinks", &PlanningSceneWrapper::getCollidingLinks)
        .def("addBox", &PlanningSceneWrapper::addBox, py::arg("id"), py::arg("size"), py::arg("pose"), py::arg("frame_id") = "world", py::arg("attach_to_link") = "", py::arg("touch_links") = std::vector<std::string>())
        .def("addSphere", &PlanningSceneWrapper::addSphere, py::arg("id"), py::arg("radius"), py::arg("pose"), py::arg("frame_id") = "world", py::arg("attach_to_link") = "", py::arg("touch_links") = std::vector<std::string>())
        .def("addCylinder", &PlanningSceneWrapper::addCylinder, py::arg("id"), py::arg("radius"), py::arg("height"), py::arg("pose"), py::arg("frame_id") = "world", py::arg("attach_to_link") = "", py::arg("touch_links") = std::vector<std::string>())
        .def("removeCollisionObject", remove_collision_object)
        .def("update", &PlanningSceneWrapper::update)
        .def("getCollidingLinks", &PlanningSceneWrapper::getCollidingLinks);

    py::class_<CentroidalStatics>(m, "CentroidalStatics")
        .def(py::init<ModelInterface::ConstPtr,
                      const std::vector<std::string>&,
                      const double,
                      const bool,
                      const Eigen::Vector2d&,
                      const Eigen::Vector2d&,
                      bool>(),
             py::arg("model"),
             py::arg("contact_links"),
             py::arg("friction_coeff"),
             py::arg("optimize_torque") = false,
             py::arg("xlims_cop") = Eigen::Vector2d::Zero(2),
             py::arg("ylims_cop") = Eigen::Vector2d::Zero(2),
             py::arg("log") = false)
        .def("checkStability", &CentroidalStatics::checkStability, py::arg("eps") = 1e-3)
        .def("setContactLinks", set_contact_links, py::arg("active_links"), py::arg("log") = false)
        .def("setContactLinks", set_contact_links_and_optimize_torque, py::arg("active_links"), py::arg("optimize_torque"), py::arg("log") = false)
        .def("getContactLinks", &CentroidalStatics::getContactLinks)
        .def("setContactRotationMatrix", &CentroidalStatics::setContactRotationMatrix)
        .def("getContactFrame", &CentroidalStatics::getContactFrame)
        .def("setForces", &CentroidalStatics::setForces)
        .def("getForces", &CentroidalStatics::getForces)
        .def("setOptimizeTorque", set_optimize_torque, py::arg("optimize_torque"), py::arg("log") = false)
        .def("isTorqueOptimized", &CentroidalStatics::isTorqueOptimized);

    py::class_<ValidityCheckContext>(m, "ValidityCheckContext")
        .def(py::init(create_validity_check_context),
             py::arg("vc_node"),
             py::arg("model"))
        .def("setPlanningScene", &ValidityCheckContext::setPlanningScene)
        .def("checkAll", &ValidityCheckContext::checkAll,
             py::arg("verbose") = false)
        .def("addChecker", add_checker)
        .def("removeChecker", rm_checker)
        .def_readwrite("planning_scene", &ValidityCheckContext::planning_scene)
        .def_readwrite("vc_aggregate", &ValidityCheckContext::vc_aggregate);
}
