#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "planner/cartesio_ompl_planner.h"
#include "utils/robot_viz.h"

#include <ros/serialization.h>
#include <eigen_conversions/eigen_msg.h>

namespace py = pybind11;

using namespace XBot;
using namespace XBot::Cartesian;
using namespace XBot::Cartesian::Planning;

auto create_robotviz = [](ModelInterface::ConstPtr model,
                        std::string topic_name,
                        const Eigen::Vector4d& color,
                        std::string prefix)
{
    ros::NodeHandle nh;
    auto ret = RobotViz(model, topic_name, nh, color);
    ret.setPrefix(prefix);
    return ret;
};

auto pub_markers = [](RobotViz& self,
                      std::vector<std::string> red_links)
{
    self.publishMarkers(ros::Time::now(), red_links);
};

PYBIND11_MODULE(visual_tools, m)
{

    py::class_<RobotViz>(m, "RobotViz")
            .def(py::init(create_robotviz),
                 py::arg("model"),
                 py::arg("topic"),
                 py::arg("color") = Eigen::Vector4d(0.0, 1.0, 0.0, 0.5),
                 py::arg("tf_prefix") = "")
            .def("publishMarkers", pub_markers, py::arg("red_links") = std::vector<std::string>())
            ;
}
