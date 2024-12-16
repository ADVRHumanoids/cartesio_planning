cmake_minimum_required(VERSION 3.20)

project(cartesio_planning VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

include(cmake/ExportProject.cmake)

find_package(cartesian_interface REQUIRED)

find_package(ompl REQUIRED)

add_library(cartesio_planning_library SHARED
    include/cartesio_planning/state_space.h
    include/cartesio_planning/cartesio_planning.h
    include/cartesio_planning/common/types.h
    include/cartesio_planning/state_validity_checker/collision.h
    include/cartesio_planning/trajectory_interpolation.h
    src/impl/cartesio_planning.hxx
    src/impl/state_space.hxx
    src/impl/parse_utils.hxx
    src/impl/profiling.hxx
    src/cartesio_planning.cpp
    src/state_space.cpp
    src/state_validity_checker.cpp
    src/collision.cpp
    src/trajectory_interpolation.cpp
    src/ompl_replacement/AtlasStateSpace.h
    src/ompl_replacement/AtlasChart.h
    src/ompl_replacement/AtlasStateSpace.cpp
    src/ompl_replacement/AtlasChart.cpp
    include/cartesio_planning/constraints/cartesian_constraint.h
    src/cartesian_constraint.cpp
    include/cartesio_planning/constraint.h
    src/constraint.cpp
    src/impl/constraint.hxx
    include/cartesio_planning/constraints/contact_constraint.h
    src/contact_constraint.cpp
)

target_include_directories(cartesio_planning_library
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    PRIVATE
    ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(cartesio_planning_library
    PUBLIC
    cartesian_interface::CartesianInterface
    PRIVATE
    ${OMPL_LIBRARIES}
)

set_target_properties(cartesio_planning_library PROPERTIES OUTPUT_NAME "cartesio_planning")

install(TARGETS cartesio_planning_library
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)

add_subdirectory(src/ros2)

export_project()

add_subdirectory(test)

add_subdirectory(python)
