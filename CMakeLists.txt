cmake_minimum_required(VERSION 3.20)

project(cartesio_planning VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(cmake/ExportProject.cmake)

find_package(cartesian_interface REQUIRED)

find_package(ompl REQUIRED)

add_library(cartesio_planning SHARED
    src/cartesio_planning.cpp
    include/cartesio_planning/cartesio_planning.h
    src/impl/state_space.hxx
    include/cartesio_planning/state_space.h
    include/common/types.h
    src/state_space.cpp
)

target_include_directories(cartesio_planning
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>
    PRIVATE
    ${OMPL_INCLUDE_DIRS}
)

target_link_libraries(cartesio_planning
    PUBLIC
    cartesian_interface::CartesianInterface
    PRIVATE
    ${OMPL_LIBRARIES}
)

install(TARGETS cartesio_planning
    EXPORT ${PROJECT_NAME}Targets
    DESTINATION lib
)

export_project()
