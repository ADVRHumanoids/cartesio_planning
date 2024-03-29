cmake_minimum_required(VERSION 3.0)
project(cartesio_planning)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

option(COMPILE_TESTS "Compile internal tests" FALSE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    std_msgs
    trajectory_msgs
    sensor_msgs
    message_generation
    kdl_parser
    robot_state_publisher
    tf tf_conversions
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    visualization_msgs
    message_generation
    )

find_package(casadi REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(ompl REQUIRED)

find_package(cartesian_interface REQUIRED)

find_package(cartesio_acceleration_support REQUIRED)

find_package(matlogger2 REQUIRED)

FIND_PACKAGE(PCL 1.7 QUIET COMPONENTS   common
                                        filters
                                        surface
                                        search
                                        io)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    CartesioPlannerStatus.msg
    CartesioPlannerGoalStatus.msg
    SetContactFrames.msg
    CartesianTrajectory.msg
    SetGroundCheck.msg
    )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    CartesioPlanner.srv
    CartesioGoal.srv
    FootStepPlanner.srv
    )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs sensor_msgs# Or other packages containing msgs
    )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include ${OMPL_INLCUDE_DIRS}
    LIBRARIES cartesio_planning cartesian_trajectory_interpolation
    CATKIN_DEPENDS roscpp message_runtime
    DEPENDS OMPL
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( SYSTEM
    src/
    include
    ${OMPL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${cartesian_interface_INCLUDE_DIRS}
    ${cartesio_acceleration_support_INCLUDE_DIRS}
    ${casadi_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS} # note: must be the last in the include list
    )

add_library(centroidal_statics SHARED src/validity_checker/stability/centroidal_statics.cpp)
target_link_libraries(centroidal_statics PRIVATE ${cartesio_acceleration_support_LIBRARIES} ${OpenSoT_LIBRARIES} ${cartesian_interface_LIBRARIES})

add_library(trajectory_interpolation SHARED src/planner/trajectory_interpolation.cpp)
target_link_libraries( trajectory_interpolation PRIVATE casadi)

add_library(cartesian_trajectory_interpolation SHARED src/planner/cartesian_trajectory_interpolation.cpp)
target_link_libraries(cartesian_trajectory_interpolation PRIVATE trajectory_interpolation ${XBotInterface_LIBRARIES})

# Main library
add_library(cartesio_planning SHARED
    src/constraints/cartesian_constraint.cpp
    src/goal/goal_sampler.cpp
    src/goal/NSPG.cpp
    src/ik/position_ik_solver.cpp
    src/planner/cartesio_ompl_planner.cpp
    src/state_wrapper.cpp
    src/utils/robot_viz.cpp
    src/utils/map_converter.cpp
    src/validity_checker/validity_checker_context.cpp
    src/validity_checker/stability/cartesian_utils.cpp
    src/validity_checker/stability/convex_hull_utils.cpp
    src/validity_checker/stability/stability_detection.cpp
    src/validity_checker/stability/centroidal_statics.cpp
    src/validity_checker/collisions/ground_collision.cpp
    src/validity_checker/collisions/planning_scene_wrapper.cpp
    src/validity_checker/collisions/ground_collision.cpp
    src/validity_checker/validity_checker_factory.cpp
    src/validity_checker/validity_predicate_aggregate.cpp
    src/nodes/goal_generation.cpp
)

target_link_libraries(cartesio_planning
    PUBLIC
    ${OMPL_LIBRARIES}
    ${cartesian_interface_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
    matlogger2::matlogger2
    cartesian_trajectory_interpolation
    PRIVATE
    ${PCL_LIBRARIES}
    centroidal_statics
    )

target_include_directories(cartesio_planning PUBLIC ${OMPL_INCLUDE_DIRS})

# Stability test exe
add_executable(stability_test src/test_nodes/stability_test.cpp)
target_link_libraries(stability_test cartesio_planning ${XBotInterface_LIBRARIES} matlogger2::matlogger2 ${catkin_LIBRARIES})

# ompl_test_control exe
add_executable(ompl_control_test src/ompl_test_control.cpp)
target_link_libraries(ompl_control_test cartesio_planning ${XBotInterface_LIBRARIES} matlogger2::matlogger2 ${catkin_LIBRARIES})


# Previewer node
add_executable(previewer
    src/nodes/trajectory_viewer.cpp
    )

target_link_libraries(previewer
    ${cartesian_interface_LIBRARIES}
    trajectory_interpolation
    ${XBotInterface_LIBRARIES}
    ${catkin_LIBRARIES}
    matlogger2::matlogger2)
    
#Publisher node
add_executable(publisher
    src/nodes/centauro_trajectory_publisher.cpp
    )

target_link_libraries(publisher   
    ${XBotInterface_LIBRARIES}
    ${catkin_LIBRARIES}
    )

# Self collision robot node
add_executable(self_collision_robot
    src/nodes/self_collision_robot.cpp
    )

target_link_libraries(self_collision_robot ${OMPL_LIBRARIES}
    yaml-cpp
    dl
    cartesio_planning
    ${XBotInterface_LIBRARIES}
    ${catkin_LIBRARIES}
    matlogger2::matlogger2)

# goal_generation node
add_executable(goal_generation_test
        src/test_nodes/goal_generation_test.cpp
    )

target_link_libraries(goal_generation_test ${OMPL_LIBRARIES}
    yaml-cpp
    dl
    cartesio_planning
    ${cartesian_interface_LIBRARIES}
    ${XBotInterface_LIBRARIES}
    ${catkin_LIBRARIES}
    matlogger2::matlogger2)

# foot-step planner node
add_executable(foot_step_planner src/nodes/foot_step_planner_main.cpp src/nodes/foot_step_planner.cpp)

target_link_libraries(foot_step_planner ${catkin_LIBRARIES} cartesian_trajectory_interpolation cartesio_planning matlogger2::matlogger2)

# Planning scene publisher
add_executable(planning_scene_publisher src/planning_scene_publisher.cpp)
target_link_libraries(planning_scene_publisher ${catkin_LIBRARIES})

# Executor
add_executable(planning_server src/nodes/planner_executor_main.cpp src/nodes/planner_executor.cpp)
target_link_libraries(planning_server ${catkin_LIBRARIES} matlogger2::matlogger2 cartesio_planning cartesian_trajectory_interpolation)

# Octomap Collision Main
add_executable(octomap_collision_main src/utils/octomap_collision_main.cpp)
target_link_libraries(octomap_collision_main cartesio_planning ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Point Cloud test
add_executable(point_cloud_test src/test_nodes/point_cloud_test.cpp)
target_link_libraries(point_cloud_test cartesio_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Centroidal Statics test
add_executable(centroidal_statics_test src/test_nodes/centroidal_statics_test.cpp)
target_link_libraries(centroidal_statics_test centroidal_statics ${XBotInterface_LIBRARIES} ${catkin_LIBRARIES} ${cartesian_interface_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/cartesio_planning_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

# Add cmake target dependencies of the executable
# same as for the library above
add_dependencies(cartesio_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS centroidal_statics
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS previewer
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    
install(TARGETS self_collision_robot
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS planning_scene_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS cartesio_planning
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS trajectory_interpolation
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS cartesian_trajectory_interpolation
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


install(TARGETS stability_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS ompl_control_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS goal_generation_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )


install(TARGETS planning_server
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS octomap_collision_main
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS point_cloud_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS centroidal_statics_test
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(TARGETS foot_step_planner
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# Mark cpp header files for installation
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY configs
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
    FILES_MATCHING PATTERN "*.yaml"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cartesio_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_subdirectory(examples/)

add_subdirectory(bindings/python/)

# compilation flags
option(CARTESIO_PLANNING_COMPILE_TESTS "Compile internal tests" FALSE)
if(${CARTESIO_PLANNING_COMPILE_TESTS})
    enable_testing()
    add_subdirectory(tests)
endif()
