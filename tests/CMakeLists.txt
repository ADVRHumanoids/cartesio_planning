cmake_minimum_required(VERSION 2.8.11)
include(ExternalProject)

project(cartesio_planning_tests C CXX)


# compite GTest
find_package(GTest)
find_package(Threads)
find_package(Boost COMPONENTS system REQUIRED)
find_package(matlogger2 REQUIRED)

if(NOT GTEST_FOUND)
    set(gTestSource "/usr/src/gtest")
    ExternalProject_Add(GTest-ext SOURCE_DIR ${gTestSource}
                                  PREFIX "${CMAKE_CURRENT_BINARY_DIR}/external"
                                  INSTALL_COMMAND "")
    set(GTEST_LIB_DIRS "${CMAKE_CURRENT_BINARY_DIR}/external/src/GTest-ext-build/")
    set(GTEST_INCLUDE_DIRS ${gTestSource})
    set(GTEST_BOTH_LIBRARIES gtest gtest_main)
endif()

add_definitions(-DTEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/configs/")

set(TestLibs cartesio_planning ${GTEST_BOTH_LIBRARIES} ${Boost_LIBRARIES} dl)
include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

add_executable(test_basics test_basics.cpp)
target_link_libraries(test_basics ${TestLibs})
add_dependencies(test_basics GTest-ext cartesio_planning)
add_test(NAME test_basics COMMAND test_basics)

add_executable(test_projection test_projection.cpp)
target_link_libraries(test_projection ${TestLibs})
add_dependencies(test_projection GTest-ext cartesio_planning)
add_test(NAME test_projection COMMAND test_projection)

add_executable(test_manifold test_manifold.cpp)
target_link_libraries(test_manifold ${TestLibs})
add_dependencies(test_manifold GTest-ext cartesio_planning)
add_test(NAME test_manifold COMMAND test_manifold)

add_executable(test_trajectory test_trajectory.cpp)
target_link_libraries(test_trajectory ${TestLibs} ${ReflexxesTypeII_LIBRARIES} matlogger2::matlogger2 trajectory_interpolation)
add_dependencies(test_trajectory GTest-ext)
add_test(NAME test_trajectory COMMAND test_trajectory)
