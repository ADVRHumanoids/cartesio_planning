find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(visualization_msgs  REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

message(STATUS "AAA ${pcl_conversions_INCLUDE_DIRS}")

set(msg_files
  "action/PlanMotion.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES
  builtin_interfaces
  sensor_msgs
  trajectory_msgs
  geometry_msgs
)


##
add_library(cartesio_planning_ros SHARED
    planning_scene_wrapper.cpp
    impl/planning_scene_wrapper.hxx
    impl/utils.hxx
    robot_viz.cpp
    robot_viz_dummy_checker.cpp
    )


target_include_directories(cartesio_planning_ros
    PUBLIC
    PRIVATE
    ${pcl_conversions_INCLUDE_DIRS}
)

target_link_libraries(cartesio_planning_ros
    PUBLIC
    cartesio_planning_library
    PRIVATE
    ${OMPL_LIBRARIES}
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    trajectory_msgs::trajectory_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    visualization_msgs::visualization_msgs__rosidl_generator_cpp
    moveit_core::moveit_robot_model
    moveit_ros_planning::moveit_cpp
    moveit_ros_planning_interface::moveit_planning_scene_interface
    pcl_ros::pcl_ros_filters
    pcl_ros::pcl_ros_tf
    tf2_eigen::tf2_eigen
    tf2_ros::tf2_ros
)

install(TARGETS cartesio_planning_ros
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)


##
# add_executable(planning_server
#     planner_executor.h planner_executor.cpp
# )

# target_link_libraries(planning_server
#     PUBLIC
#     cartesio_planning_library
#     cartesio_planning_ros
# )

# install(TARGETS planning_server
#     EXPORT ${PROJECT_NAME}Targets
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# ##
# add_executable(goal_sampler_node
#     goal_sampler_node.cpp
# )

# target_link_libraries(goal_sampler_node
#     PUBLIC
#     cartesio_planning_library
#     cartesio_planning_ros
# )

# install(TARGETS goal_sampler_node
#     EXPORT ${PROJECT_NAME}Targets
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


