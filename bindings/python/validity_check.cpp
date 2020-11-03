#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "planner/cartesio_ompl_planner.h"
#include "validity_checker/collisions/planning_scene_wrapper.h"
#include "validity_checker/stability/centroidal_statics.h"


#include <ros/serialization.h>
#include <eigen_conversions/eigen_msg.h>

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Planning;

auto add_box = [](PlanningSceneWrapper& self,
                std::string id,
                const Eigen::Vector3d& size,
                const Eigen::Affine3d& T)
{
    moveit_msgs::CollisionObject co;
    co.id = id;
    co.header.frame_id = "/world";

    shape_msgs::SolidPrimitive solid;
    solid.type = solid.BOX;
    solid.dimensions = {size.x(), size.y(), size.z()};
    co.primitives.push_back(solid);

    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(T, pose);
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;
    ps.world.collision_objects.push_back(co);

    self.applyPlanningScene(ps);

};

auto add_ellipse = [](PlanningSceneWrapper& self,
                std::string id,
                double radius,
                const Eigen::Affine3d& T)
{
    moveit_msgs::CollisionObject co;
    co.id = id;

    shape_msgs::SolidPrimitive solid;
    solid.type = solid.SPHERE;
    solid.dimensions = {radius};
    co.primitives.push_back(solid);

    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(T, pose);
    co.primitive_poses.push_back(pose);

    co.operation = co.ADD;

    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;
    ps.world.collision_objects.push_back(co);

    self.applyPlanningScene(ps);

};


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

PYBIND11_MODULE(validity_check, m)
{

    py::class_<PlanningSceneWrapper>(m, "PlanningSceneWrapper")
            .def(py::init<ModelInterface::ConstPtr>())
            .def("startMonitor", &PlanningSceneWrapper::startMonitor)
            .def("stopMonitor", &PlanningSceneWrapper::stopMonitor)
            .def("startGetPlanningSceneServer", &PlanningSceneWrapper::startGetPlanningSceneServer)
            .def("checkCollisions", &PlanningSceneWrapper::checkCollisions)
            .def("checkSelfCollisions", &PlanningSceneWrapper::checkSelfCollisions)
            .def("getCollidingLinks", &PlanningSceneWrapper::getCollidingLinks)
            .def("addBox", add_box)
            .def("addSphere", add_ellipse)
            .def("removeCollisionObject", remove_collision_object)
            .def("update", &PlanningSceneWrapper::update)
            .def("getCollidingLinks", &PlanningSceneWrapper::getCollidingLinks);
            
    py::class_<CentroidalStatics>(m, "CentroidalStatics")
            .def(py::init<ModelInterface::ConstPtr, 
                         const std::vector<std::string>&,
                         const double,
                         const bool,
                         const Eigen::Vector2d&,
                         const Eigen::Vector2d&>(),
                 py::arg("model"),
                 py::arg("contact_links"),
                 py::arg("friction_coeff"),
                 py::arg("optimize_torque") = false,
                 py::arg("xlims_cop") = Eigen::Vector2d::Zero(2),
                 py::arg("ylims_cop") = Eigen::Vector2d::Zero(2))
            .def("checkStability", &CentroidalStatics::checkStability, py::arg("eps") = 1e-3)
            .def("setContactLinks", &CentroidalStatics::setContactLinks)
            .def("addContactLinks", &CentroidalStatics::addContactLinks)
            .def("getContactLinks", &CentroidalStatics::getContactLinks)
            .def("setContactRotationMatrix", &CentroidalStatics::setContactRotationMatrix)
            .def("getContactFrame", &CentroidalStatics::getContactFrame)
            .def("setForces", &CentroidalStatics::setForces)
            .def("getForces", &CentroidalStatics::getForces);       
}
