find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    trajectory_msgs
    sensor_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    visualization_msgs )

find_package(pcl_ros REQUIRED)

find_package(eigen_conversions REQUIRED)

find_package(tf_conversions REQUIRED)

add_action_files(
    FILES
    PlanMotion.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    trajectory_msgs
    sensor_msgs
)

catkin_package()


##
add_library(cartesio_planning_ros SHARED
    planning_scene_wrapper.cpp
    impl/planning_scene_wrapper.hxx
    impl/utils.hxx
    robot_viz.cpp
    robot_viz_dummy_checker.cpp
    )

add_dependencies(cartesio_planning_ros ${PROJECT_NAME}_generate_messages)

target_include_directories(cartesio_planning_ros
    PUBLIC
    PRIVATE
    ${pcl_ros_INCLUDE_DIRS}
    ${eigen_conversions_INCLUDE_DIRS}
    ${tf_conversions_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(cartesio_planning_ros
    PUBLIC
    cartesio_planning
    PRIVATE
    ${OMPL_LIBRARIES}
    ${pcl_ros_LIBRARIES}
    ${catkin_LIBRARIES}
)

install(TARGETS cartesio_planning_ros
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)


##
add_executable(planning_server
    planner_executor.h planner_executor.cpp
)

target_link_libraries(planning_server
    PUBLIC
    cartesio_planning
    cartesio_planning_ros
)

install(TARGETS planning_server
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

